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

#include <dart/constraint/ConstrainedGroup.hpp>
#include <dart/constraint/ContactConstraint.hpp>
#include <dart/constraint/ContactSurface.hpp>
#include <dart/constraint/ExactCoulombFbfConstraintSolver.hpp>

#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/Contact.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <memory>
#include <new>
#include <string>

#include <cmath>

namespace {

class ExposedExactCoulombFbfConstraintSolver final
  : public dart::constraint::ExactCoulombFbfConstraintSolver
{
public:
  using ExactCoulombFbfConstraintSolver::ExactCoulombFbfConstraintSolver;
  using ExactCoulombFbfConstraintSolver::solveConstrainedGroup;
};

class ContactLikeConstraint final : public dart::constraint::ConstraintBase
{
public:
  ContactLikeConstraint(
      Eigen::Index offset,
      const Eigen::MatrixXd& delassus,
      const Eigen::Vector3d& rhs,
      double primaryCoefficient,
      double secondaryCoefficient,
      std::shared_ptr<Eigen::Index> activeRow)
    : mOffset(offset),
      mDelassus(delassus),
      mRhs(rhs),
      mPrimaryCoefficient(primaryCoefficient),
      mSecondaryCoefficient(secondaryCoefficient),
      mActiveRow(std::move(activeRow))
  {
    mDim = 3u;
    mAppliedImpulse.setZero();
  }

  void update() override {}

  void getInformation(dart::constraint::ConstraintInfo* info) override
  {
    info->x[0] = 0.0;
    info->x[1] = 0.0;
    info->x[2] = 0.0;

    info->lo[0] = 0.0;
    info->lo[1] = -mPrimaryCoefficient;
    info->lo[2] = -mSecondaryCoefficient;

    info->hi[0] = std::numeric_limits<double>::infinity();
    info->hi[1] = mPrimaryCoefficient;
    info->hi[2] = mSecondaryCoefficient;

    info->b[0] = mRhs[0];
    info->b[1] = mRhs[1];
    info->b[2] = mRhs[2];

    info->w[0] = 0.0;
    info->w[1] = 0.0;
    info->w[2] = 0.0;

    info->findex[0] = -1;
    info->findex[1] = 0;
    info->findex[2] = 0;
  }

  void applyUnitImpulse(std::size_t index) override
  {
    *mActiveRow = mOffset + static_cast<Eigen::Index>(index);
  }

  void getVelocityChange(double* velocityChange, bool) override
  {
    Eigen::Map<Eigen::Vector3d> velocity(velocityChange);
    velocity.setZero();
    if (*mActiveRow < 0) {
      return;
    }

    velocity = mDelassus.block<1, 3>(*mActiveRow, mOffset).transpose();
  }

  void excite() override {}

  void unexcite() override
  {
    *mActiveRow = -1;
  }

  void applyImpulse(double* lambda) override
  {
    mAppliedImpulse = Eigen::Map<Eigen::Vector3d>(lambda);
  }

  bool isActive() const override
  {
    return true;
  }

  dart::dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return nullptr;
  }

  const Eigen::Vector3d& getAppliedImpulse() const
  {
    return mAppliedImpulse;
  }

  void setRhs(const Eigen::Vector3d& rhs)
  {
    mRhs = rhs;
  }

private:
  Eigen::Index mOffset;
  Eigen::MatrixXd mDelassus;
  Eigen::Vector3d mRhs;
  double mPrimaryCoefficient;
  double mSecondaryCoefficient;
  std::shared_ptr<Eigen::Index> mActiveRow;
  Eigen::Vector3d mAppliedImpulse;
};

class ScalarConstraint final : public dart::constraint::ConstraintBase
{
public:
  ScalarConstraint()
  {
    mDim = 1u;
  }

  void update() override {}

  void getInformation(dart::constraint::ConstraintInfo* info) override
  {
    info->x[0] = 0.0;
    info->lo[0] = 0.0;
    info->hi[0] = std::numeric_limits<double>::infinity();
    info->b[0] = -1.0;
    info->w[0] = 0.0;
    info->findex[0] = -1;
  }

  void applyUnitImpulse(std::size_t) override
  {
    mImpulseApplied = true;
  }

  void getVelocityChange(double* velocityChange, bool) override
  {
    velocityChange[0] = mImpulseApplied ? 1.0 : 0.0;
  }

  void excite() override
  {
    mImpulseApplied = false;
  }

  void unexcite() override
  {
    mImpulseApplied = false;
  }

  void applyImpulse(double* lambda) override
  {
    mApplied = true;
    mAppliedImpulse = lambda[0];
  }

  bool isActive() const override
  {
    return true;
  }

  dart::dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return nullptr;
  }

  bool wasApplied() const
  {
    return mApplied;
  }

  double getAppliedImpulse() const
  {
    return mAppliedImpulse;
  }

private:
  bool mImpulseApplied = false;
  bool mApplied = false;
  double mAppliedImpulse = 0.0;
};

class FakeCollisionObject final : public dart::collision::CollisionObject
{
public:
  FakeCollisionObject(
      dart::collision::CollisionDetector* detector,
      const dart::dynamics::ShapeFrame* shapeFrame)
    : dart::collision::CollisionObject(detector, shapeFrame)
  {
  }

protected:
  void updateEngineData() override {}
};

class FakeCollisionDetector final : public dart::collision::CollisionDetector
{
public:
  std::shared_ptr<dart::collision::CollisionDetector>
  cloneWithoutCollisionObjects() const override
  {
    return std::make_shared<FakeCollisionDetector>();
  }

  const std::string& getType() const override
  {
    static const std::string type = "FakeCollisionDetector";
    return type;
  }

  std::unique_ptr<dart::collision::CollisionGroup> createCollisionGroup()
      override
  {
    return nullptr;
  }

  bool collide(
      dart::collision::CollisionGroup*,
      const dart::collision::CollisionOption& = dart::collision::
          CollisionOption(),
      dart::collision::CollisionResult* = nullptr) override
  {
    return false;
  }

  bool collide(
      dart::collision::CollisionGroup*,
      dart::collision::CollisionGroup*,
      const dart::collision::CollisionOption& = dart::collision::
          CollisionOption(),
      dart::collision::CollisionResult* = nullptr) override
  {
    return false;
  }

  double distance(
      dart::collision::CollisionGroup*,
      const dart::collision::DistanceOption& = dart::collision::
          DistanceOption(),
      dart::collision::DistanceResult* = nullptr) override
  {
    return 0.0;
  }

  double distance(
      dart::collision::CollisionGroup*,
      dart::collision::CollisionGroup*,
      const dart::collision::DistanceOption& = dart::collision::
          DistanceOption(),
      dart::collision::DistanceResult* = nullptr) override
  {
    return 0.0;
  }

protected:
  std::unique_ptr<dart::collision::CollisionObject> createCollisionObject(
      const dart::dynamics::ShapeFrame*) override
  {
    return nullptr;
  }

  void refreshCollisionObject(dart::collision::CollisionObject*) override {}
};

struct FreeContactBody
{
  dart::dynamics::SkeletonPtr skeleton;
  dart::dynamics::FreeJoint* joint = nullptr;
  dart::dynamics::BodyNode* body = nullptr;
  dart::dynamics::ShapeNode* shape = nullptr;
};

FreeContactBody createFreeContactBody(const std::string& name, bool mobile)
{
  FreeContactBody result;
  result.skeleton = dart::dynamics::Skeleton::create(name);
  const auto pair
      = result.skeleton
            ->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  result.joint = pair.first;
  result.body = pair.second;
  result.shape = result.body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d::Ones()));
  result.skeleton->setMobile(mobile);
  return result;
}

dart::collision::Contact makePhysicalContact(
    dart::collision::CollisionObject* objectA,
    dart::collision::CollisionObject* objectB,
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& normal)
{
  dart::collision::Contact contact;
  contact.collisionObject1 = objectA;
  contact.collisionObject2 = objectB;
  contact.point = point;
  contact.normal = normal;
  contact.penetrationDepth = 0.0;
  return contact;
}

std::shared_ptr<dart::constraint::ContactConstraint> makePhysicalConstraint(
    dart::collision::Contact& contact)
{
  auto constraint = std::make_shared<dart::constraint::ContactConstraint>(
      contact, 0.001, dart::constraint::ContactSurfaceParams{});
  dart::constraint::ConstraintBase& base = *constraint;
  base.update();
  return constraint;
}

dart::constraint::ExactCoulombFbfConstraintSolverOptions
makeZeroBudgetWarmStartOptions()
{
  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 0;
  options.tolerance = 0.0;
  options.seedNormalImpulseFromDiagonal = false;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.fallbackToBoxedLcp = false;
  return options;
}

dart::constraint::ConstrainedGroup makeGroup(
    const std::shared_ptr<dart::constraint::ConstraintBase>& constraint)
{
  dart::constraint::ConstrainedGroup group;
  group.addConstraint(constraint);
  return group;
}

} // namespace

TEST(ExactCoulombFbfConstraintSolver, SolvesSupportedContactGroup)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;
  options.seedNormalImpulseFromDiagonal = false;
  options.maxResidualHistorySamples = 128;
  options.maxResidualHistoryRecords = 4;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_FALSE(solver.getLastExactCoulombProjectedGradientRetryUsed());
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 1u);
  EXPECT_EQ(solver.getNumExactCoulombAttempts(), 1u);
  EXPECT_EQ(solver.getNumExactCoulombMaxIterationsAccepted(), 0u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombProjectedGradientRetries(), 0u);
  EXPECT_NEAR(solver.getLastExactCoulombResidual(), 0.0, 1e-10);
  EXPECT_NEAR(
      solver.getWorstExactCoulombResidual(),
      solver.getLastExactCoulombResidual(),
      1e-10);
  EXPECT_TRUE(std::isfinite(solver.getLastExactCoulombBestResidual()));
  EXPECT_LE(
      solver.getLastExactCoulombBestResidual(),
      solver.getLastExactCoulombResidual());
  EXPECT_LE(
      solver.getLastExactCoulombBestIteration(),
      solver.getLastExactCoulombIterations());
  EXPECT_NEAR(solver.getLastExactCoulombResidualDetails().value, 0.0, 1e-10);
  EXPECT_NEAR(
      solver.getLastExactCoulombResidualDetails().primalFeasibility,
      0.0,
      1e-10);
  EXPECT_NEAR(
      solver.getLastExactCoulombResidualDetails().dualFeasibility, 0.0, 1e-10);
  EXPECT_NEAR(
      solver.getLastExactCoulombResidualDetails().complementarity, 0.0, 1e-10);
  const auto& residualHistory = solver.getLastExactCoulombResidualHistory();
  ASSERT_GE(residualHistory.size(), 2u);
  EXPECT_EQ(residualHistory.front().iteration, 0);
  EXPECT_EQ(
      residualHistory.back().iteration, solver.getLastExactCoulombIterations());
  EXPECT_GT(
      residualHistory.front().residual.value,
      residualHistory.back().residual.value);
  EXPECT_NEAR(
      residualHistory.back().residual.value,
      solver.getLastExactCoulombResidual(),
      1e-10);

  const auto& residualHistoryRecords
      = solver.getExactCoulombResidualHistoryRecords();
  ASSERT_EQ(residualHistoryRecords.size(), 1u);
  EXPECT_EQ(residualHistoryRecords.front().solveIndex, 0u);
  EXPECT_EQ(residualHistoryRecords.front().contactCount, 1u);
  EXPECT_EQ(
      residualHistoryRecords.front().status,
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_EQ(
      residualHistoryRecords.front().fbfStatus,
      dart::math::detail::ExactCoulombFbfStatus::Success);
  EXPECT_EQ(
      residualHistoryRecords.front().iterations,
      solver.getLastExactCoulombIterations());
  ASSERT_EQ(
      residualHistoryRecords.front().samples.size(), residualHistory.size());
  EXPECT_NEAR(
      residualHistoryRecords.front().samples.back().residual.value,
      solver.getLastExactCoulombResidual(),
      1e-10);

  solver.clearExactCoulombResidualHistoryRecords();
  EXPECT_TRUE(solver.getExactCoulombResidualHistoryRecords().empty());

  EXPECT_TRUE(std::isfinite(solver.getLastExactCoulombStepSize()));
  EXPECT_TRUE(std::isfinite(solver.getLastExactCoulombSafeStepSize()));
  EXPECT_TRUE(
      std::isfinite(solver.getLastExactCoulombCouplingVariationRatio()));
  EXPECT_GE(solver.getLastExactCoulombShrinkIterations(), 0);
  EXPECT_EQ(
      solver.getMaxExactCoulombIterations(),
      solver.getLastExactCoulombIterations());
  EXPECT_EQ(
      solver.getTotalExactCoulombIterations(),
      static_cast<std::size_t>(solver.getLastExactCoulombIterations()));
  EXPECT_NEAR(contact->getAppliedImpulse()[0], 1.0, 1e-9);
  EXPECT_NEAR(contact->getAppliedImpulse()[1], 0.0, 1e-12);
  EXPECT_NEAR(contact->getAppliedImpulse()[2], 0.0, 1e-12);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    CanUseMatrixFreeDelassusOperatorForSupportedContactGroup)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;
  options.useMatrixFreeDelassusOperator = true;
  options.useMatrixFreeDelassusSeed = true;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombMatrixFreeDelassusOperatorUsed());
  EXPECT_TRUE(solver.getLastExactCoulombMatrixFreeDelassusSeedUsed());
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 1u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_NEAR(solver.getLastExactCoulombResidual(), 0.0, 1e-10);
  EXPECT_NEAR(contact->getAppliedImpulse()[0], 1.0, 1e-9);
  EXPECT_NEAR(contact->getAppliedImpulse()[1], 0.0, 1e-12);
  EXPECT_NEAR(contact->getAppliedImpulse()[2], 0.0, 1e-12);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    RetriesProjectedGradientWhenBlockGaussSeidelFails)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;
  options.innerMaxSweeps = 0;
  options.projectedGradientMaxIterations = 200;
  options.projectedGradientTolerance = 1e-14;
  options.seedNormalImpulseFromDiagonal = false;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombProjectedGradientRetryUsed());
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 1u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombProjectedGradientRetries(), 1u);
  EXPECT_NEAR(solver.getLastExactCoulombResidual(), 0.0, 1e-10);
  EXPECT_NEAR(solver.getLastExactCoulombResidualDetails().value, 0.0, 1e-10);
  EXPECT_NEAR(contact->getAppliedImpulse()[0], 1.0, 1e-9);
  EXPECT_NEAR(contact->getAppliedImpulse()[1], 0.0, 1e-12);
  EXPECT_NEAR(contact->getAppliedImpulse()[2], 0.0, 1e-12);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    AcceptsFiniteIterateFromFixedInnerSweepBudget)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;
  options.innerMaxSweeps = 1;
  options.innerLocalIterations = 1;
  options.innerTolerance = 0.0;
  options.seedNormalImpulseFromDiagonal = false;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.fallbackToBoxedLcp = false;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 1u);
  EXPECT_EQ(solver.getNumExactCoulombAttempts(), 1u);
  EXPECT_EQ(solver.getNumExactCoulombMaxIterationsAccepted(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombFailures(), 0u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_LE(solver.getLastExactCoulombResidual(), options.tolerance);
  EXPECT_NEAR(contact->getAppliedImpulse()[0], 1.0, 1e-9);
  EXPECT_NEAR(contact->getAppliedImpulse()[1], 0.0, 1e-12);
  EXPECT_NEAR(contact->getAppliedImpulse()[2], 0.0, 1e-12);
}

TEST(ExactCoulombFbfConstraintSolver, CanRequireEveryInnerSolveToConverge)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;
  options.innerMaxSweeps = 1;
  options.innerLocalIterations = 1;
  options.innerTolerance = 0.0;
  options.acceptInnerMaxIterations = false;
  options.seedNormalImpulseFromDiagonal = false;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.fallbackToBoxedLcp = false;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::FbfFailed);
  EXPECT_EQ(
      solver.getLastFailedExactCoulombFbfStatus(),
      dart::math::detail::ExactCoulombFbfStatus::InnerSolverFailed);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombFailures(), 1u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_TRUE(contact->getAppliedImpulse().isZero());
}

TEST(
    ExactCoulombFbfConstraintSolver,
    CanApplyFiniteReactionAtOuterIterationBudget)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 0;
  options.tolerance = 0.0;
  options.acceptOuterMaxIterations = true;
  options.seedNormalImpulseFromDiagonal = false;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.fallbackToBoxedLcp = false;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::
          MaxIterationsAccepted);
  EXPECT_EQ(
      solver.getLastExactCoulombFbfStatus(),
      dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 1u);
  EXPECT_EQ(solver.getNumExactCoulombAttempts(), 1u);
  EXPECT_EQ(solver.getNumExactCoulombMaxIterationsAccepted(), 1u);
  EXPECT_EQ(solver.getNumExactCoulombFailures(), 0u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_FALSE(solver.getLastExactCoulombProjectedGradientRetryUsed());
  EXPECT_EQ(solver.getNumExactCoulombProjectedGradientRetries(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombPersistentStepSizeRetries(), 0u);
  EXPECT_GT(solver.getLastExactCoulombResidual(), options.tolerance);
  const double firstCappedResidual = solver.getLastExactCoulombResidual();
  EXPECT_EQ(solver.getWorstExactCoulombResidual(), firstCappedResidual);

  // A persisted hint on the next capped solve must not trigger a hidden fresh
  // automatic retry or silently double the configured outer budget.
  solver.solveConstrainedGroup(group);
  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::
          MaxIterationsAccepted);
  EXPECT_TRUE(solver.getLastExactCoulombPersistentStepSizeUsed());
  EXPECT_EQ(solver.getNumExactCoulombAttempts(), 2u);
  EXPECT_EQ(solver.getNumExactCoulombMaxIterationsAccepted(), 2u);
  EXPECT_EQ(
      solver.getWorstExactCoulombResidual(),
      std::max(firstCappedResidual, solver.getLastExactCoulombResidual()));
  EXPECT_EQ(solver.getNumExactCoulombPersistentStepSizeRetries(), 0u);
  EXPECT_EQ(solver.getTotalExactCoulombIterations(), 0u);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    MatrixFreeDelassusSeedDiagnosticYieldsToWarmStart)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;
  options.useMatrixFreeDelassusOperator = true;
  options.useMatrixFreeDelassusSeed = true;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombMatrixFreeDelassusSeedUsed());
  EXPECT_FALSE(solver.getLastExactCoulombWarmStartUsed());

  options.maxOuterIterations = 0;
  solver.setExactCoulombOptions(options);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_FALSE(solver.getLastExactCoulombMatrixFreeDelassusSeedUsed());
  EXPECT_EQ(solver.getNumExactCoulombWarmStarts(), 1u);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    RestoresDenseColdSeedAfterUnsupportedContactRowFallback)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto fallbackContact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto fallbackGroup = makeGroup(fallbackContact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions fallbackOptions;
  fallbackOptions.maxOuterIterations = 0;
  fallbackOptions.tolerance = 0.0;
  fallbackOptions.fallbackToBoxedLcp = false;
  fallbackOptions.enableProjectedGradientRetry = false;
  fallbackOptions.enableDenseResidualPolish = false;
  ASSERT_TRUE(fallbackOptions.useContactRowDelassusOperator);
  ASSERT_TRUE(fallbackOptions.seedNormalImpulseFromDiagonal);

  ExposedExactCoulombFbfConstraintSolver fallbackSolver(fallbackOptions);
  fallbackSolver.setTimeStep(0.001);
  fallbackSolver.solveConstrainedGroup(fallbackGroup);

  EXPECT_EQ(
      fallbackSolver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_FALSE(fallbackSolver.getLastExactCoulombContactRowOperatorUsed());
  EXPECT_EQ(fallbackSolver.getLastExactCoulombIterations(), 0);
  EXPECT_NEAR(fallbackContact->getAppliedImpulse()[0], 1.0, 1e-12);

  auto denseContact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto denseGroup = makeGroup(denseContact);
  auto denseOptions = fallbackOptions;
  denseOptions.useContactRowDelassusOperator = false;
  ExposedExactCoulombFbfConstraintSolver denseSolver(denseOptions);
  denseSolver.setTimeStep(0.001);
  denseSolver.solveConstrainedGroup(denseGroup);

  EXPECT_EQ(
      denseSolver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_EQ(denseSolver.getLastExactCoulombIterations(), 0);
  EXPECT_TRUE(denseContact->getAppliedImpulse().isApprox(
      fallbackContact->getAppliedImpulse(), 1e-12));
}

TEST(ExactCoulombFbfConstraintSolver, ReusesWarmStartForSameConstraintSequence)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_FALSE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_EQ(solver.getNumExactCoulombWarmStarts(), 0u);

  options.maxOuterIterations = 0;
  solver.setExactCoulombOptions(options);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_EQ(solver.getNumExactCoulombWarmStarts(), 1u);
  EXPECT_EQ(solver.getLastExactCoulombIterations(), 0);
  EXPECT_NEAR(solver.getLastExactCoulombResidual(), 0.0, 1e-10);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 2u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_NEAR(contact->getAppliedImpulse()[0], 1.0, 1e-9);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    BodyLocalWarmStartSurvivesWorldMotionFromRigidRotation)
{
  FakeCollisionDetector detector;
  auto dynamicBody = createFreeContactBody("dynamic", true);
  auto fixedBody = createFreeContactBody("fixed", false);
  FakeCollisionObject dynamicObject(&detector, dynamicBody.shape);
  FakeCollisionObject fixedObject(&detector, fixedBody.shape);

  const Eigen::Vector3d firstPoint(0.2, 0.0, 0.0);
  const Eigen::Vector3d firstNormal = Eigen::Vector3d::UnitX();
  auto firstContact = makePhysicalContact(
      &dynamicObject, &fixedObject, firstPoint, firstNormal);
  auto firstGroup = makeGroup(makePhysicalConstraint(firstContact));

  const auto options = makeZeroBudgetWarmStartOptions();
  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(firstGroup);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  ASSERT_FALSE(solver.getLastExactCoulombWarmStartUsed());

  Eigen::Isometry3d rotatedTransform = Eigen::Isometry3d::Identity();
  rotatedTransform.linear()
      = Eigen::AngleAxisd(0.5 * std::acos(-1.0), Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
  dynamicBody.joint->setTransform(rotatedTransform);
  const Eigen::Vector3d secondPoint = rotatedTransform * firstPoint;
  const Eigen::Vector3d secondNormal = rotatedTransform.linear() * firstNormal;
  ASSERT_GT((secondPoint - firstPoint).norm(), options.warmStartMatchDistance);

  auto secondContact = makePhysicalContact(
      &dynamicObject, &fixedObject, secondPoint, secondNormal);
  auto secondGroup = makeGroup(makePhysicalConstraint(secondContact));
  solver.solveConstrainedGroup(secondGroup);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_TRUE(solver.getLastExactCoulombPersistentStepSizeUsed());
  EXPECT_EQ(solver.getLastExactCoulombWarmStartMatchedContacts(), 1u);
}

TEST(
    ExactCoulombFbfConstraintSolver, BodyLocalWarmStartMatchesReversedBodyOrder)
{
  FakeCollisionDetector detector;
  auto dynamicBody = createFreeContactBody("dynamic", true);
  auto fixedBody = createFreeContactBody("fixed", false);
  FakeCollisionObject dynamicObject(&detector, dynamicBody.shape);
  FakeCollisionObject fixedObject(&detector, fixedBody.shape);

  const Eigen::Vector3d point(0.2, 0.0, 0.0);
  auto firstContact = makePhysicalContact(
      &dynamicObject, &fixedObject, point, Eigen::Vector3d::UnitX());
  auto firstGroup = makeGroup(makePhysicalConstraint(firstContact));

  const auto options = makeZeroBudgetWarmStartOptions();
  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(firstGroup);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);

  auto reversedContact = makePhysicalContact(
      &fixedObject, &dynamicObject, point, -Eigen::Vector3d::UnitX());
  auto reversedGroup = makeGroup(makePhysicalConstraint(reversedContact));
  solver.solveConstrainedGroup(reversedGroup);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_TRUE(solver.getLastExactCoulombPersistentStepSizeUsed());
  EXPECT_EQ(solver.getLastExactCoulombWarmStartMatchedContacts(), 1u);
}

TEST(
    ExactCoulombFbfConstraintSolver, OrderedBodyBPolicyRejectsReversedBodyOrder)
{
  FakeCollisionDetector detector;
  auto dynamicBody = createFreeContactBody("dynamic", true);
  auto fixedBody = createFreeContactBody("fixed", false);
  FakeCollisionObject dynamicObject(&detector, dynamicBody.shape);
  FakeCollisionObject fixedObject(&detector, fixedBody.shape);

  const Eigen::Vector3d point(0.2, 0.0, 0.0);
  auto firstContact = makePhysicalContact(
      &dynamicObject, &fixedObject, point, Eigen::Vector3d::UnitX());
  auto firstGroup = makeGroup(makePhysicalConstraint(firstContact));

  const auto options = makeZeroBudgetWarmStartOptions();
  ExposedExactCoulombFbfConstraintSolver solver(options);
  dart::constraint::ExactCoulombFbfCrossStepPolicyOptions policy;
  policy.warmStartMatchMode = dart::constraint::
      ExactCoulombFbfWarmStartMatchMode::OrderedBodyBLocalFeature;
  policy.useStrictWarmStartMatchDistance = true;
  solver.setExactCoulombCrossStepPolicyOptions(policy);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(firstGroup);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);

  auto reversedContact = makePhysicalContact(
      &fixedObject, &dynamicObject, point, -Eigen::Vector3d::UnitX());
  auto reversedGroup = makeGroup(makePhysicalConstraint(reversedContact));
  solver.solveConstrainedGroup(reversedGroup);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_FALSE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_FALSE(solver.getLastExactCoulombPersistentStepSizeUsed());
  EXPECT_EQ(solver.getLastExactCoulombWarmStartMatchedContacts(), 0u);
}

TEST(ExactCoulombFbfConstraintSolver, BodyLocalWarmStartRejectsNormalMismatch)
{
  FakeCollisionDetector detector;
  auto dynamicBody = createFreeContactBody("dynamic", true);
  auto fixedBody = createFreeContactBody("fixed", false);
  FakeCollisionObject dynamicObject(&detector, dynamicBody.shape);
  FakeCollisionObject fixedObject(&detector, fixedBody.shape);

  const Eigen::Vector3d point(0.2, 0.0, 0.0);
  auto firstContact = makePhysicalContact(
      &dynamicObject, &fixedObject, point, Eigen::Vector3d::UnitX());
  auto firstGroup = makeGroup(makePhysicalConstraint(firstContact));

  const auto options = makeZeroBudgetWarmStartOptions();
  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(firstGroup);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);

  auto changedNormalContact = makePhysicalContact(
      &dynamicObject, &fixedObject, point, Eigen::Vector3d::UnitY());
  auto changedNormalGroup
      = makeGroup(makePhysicalConstraint(changedNormalContact));
  solver.solveConstrainedGroup(changedNormalGroup);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_FALSE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_FALSE(solver.getLastExactCoulombPersistentStepSizeUsed());
  EXPECT_EQ(solver.getLastExactCoulombWarmStartMatchedContacts(), 0u);
}

TEST(ExactCoulombFbfConstraintSolver, WarmStartRequiresSameConstraintSequence)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto first = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto firstGroup = makeGroup(first);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(firstGroup);

  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);

  auto second = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto secondGroup = makeGroup(second);

  options.maxOuterIterations = 0;
  options.fallbackToBoxedLcp = false;
  options.seedNormalImpulseFromDiagonal = false;
  options.enableDenseResidualPolish = false;
  solver.setExactCoulombOptions(options);
  solver.solveConstrainedGroup(secondGroup);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::FbfFailed);
  EXPECT_EQ(
      solver.getLastExactCoulombFbfStatus(),
      dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  EXPECT_EQ(
      solver.getLastFailedExactCoulombFbfStatus(),
      dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  EXPECT_EQ(
      solver.getLastFailedExactCoulombIterations(),
      solver.getLastExactCoulombIterations());
  EXPECT_EQ(
      solver.getLastFailedExactCoulombStepSize(),
      solver.getLastExactCoulombStepSize());
  EXPECT_EQ(
      solver.getLastFailedExactCoulombSafeStepSize(),
      solver.getLastExactCoulombSafeStepSize());
  EXPECT_EQ(
      solver.getLastFailedExactCoulombCouplingVariationRatio(),
      solver.getLastExactCoulombCouplingVariationRatio());
  EXPECT_EQ(
      solver.getLastFailedExactCoulombShrinkIterations(),
      solver.getLastExactCoulombShrinkIterations());
  EXPECT_EQ(
      solver.getLastFailedExactCoulombResidual(),
      solver.getLastExactCoulombResidual());
  EXPECT_EQ(
      solver.getLastFailedExactCoulombResidualDetails().value,
      solver.getLastExactCoulombResidualDetails().value);
  EXPECT_EQ(solver.getLastFailedExactCoulombContactCount(), 1u);
  EXPECT_EQ(
      solver.getLastFailedExactCoulombBestResidual(),
      solver.getLastExactCoulombBestResidual());
  EXPECT_EQ(
      solver.getLastFailedExactCoulombBestIteration(),
      solver.getLastExactCoulombBestIteration());
  EXPECT_FALSE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_EQ(solver.getNumExactCoulombWarmStarts(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 1u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_NEAR(second->getAppliedImpulse().norm(), 0.0, 1e-12);

  const auto failedStatus = solver.getLastFailedExactCoulombStatus();
  const auto failedBuildStatus = solver.getLastFailedExactCoulombBuildStatus();
  const auto failedFbfStatus = solver.getLastFailedExactCoulombFbfStatus();
  const auto failedContactCount
      = solver.getLastFailedExactCoulombContactCount();
  const auto failedResidual = solver.getLastFailedExactCoulombResidual();
  const auto failedResidualDetails
      = solver.getLastFailedExactCoulombResidualDetails();
  const auto failedBestResidual
      = solver.getLastFailedExactCoulombBestResidual();
  const auto failedBestIteration
      = solver.getLastFailedExactCoulombBestIteration();
  const auto failedIterations = solver.getLastFailedExactCoulombIterations();
  const auto failedStepSize = solver.getLastFailedExactCoulombStepSize();
  const auto failedSafeStepSize
      = solver.getLastFailedExactCoulombSafeStepSize();
  const auto failedCouplingVariation
      = solver.getLastFailedExactCoulombCouplingVariationRatio();
  const auto failedShrinkIterations
      = solver.getLastFailedExactCoulombShrinkIterations();
  EXPECT_EQ(
      failedStatus,
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::FbfFailed);
  EXPECT_EQ(
      failedBuildStatus,
      dart::constraint::detail::ExactCoulombConstraintBuildStatus::Success);

  auto zeroRhs = std::make_shared<ContactLikeConstraint>(
      0,
      delassus,
      Eigen::Vector3d::Zero(),
      0.5,
      0.5,
      std::make_shared<Eigen::Index>(-1));
  auto successfulGroup = makeGroup(zeroRhs);
  options.maxOuterIterations = 80;
  solver.setExactCoulombOptions(options);
  solver.solveConstrainedGroup(successfulGroup);

  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_EQ(solver.getLastFailedExactCoulombStatus(), failedStatus);
  EXPECT_EQ(solver.getLastFailedExactCoulombBuildStatus(), failedBuildStatus);
  EXPECT_EQ(solver.getLastFailedExactCoulombFbfStatus(), failedFbfStatus);
  EXPECT_EQ(solver.getLastFailedExactCoulombContactCount(), failedContactCount);
  EXPECT_EQ(solver.getLastFailedExactCoulombResidual(), failedResidual);
  const auto& retainedResidual
      = solver.getLastFailedExactCoulombResidualDetails();
  EXPECT_EQ(
      retainedResidual.primalFeasibility,
      failedResidualDetails.primalFeasibility);
  EXPECT_EQ(
      retainedResidual.dualFeasibility, failedResidualDetails.dualFeasibility);
  EXPECT_EQ(
      retainedResidual.complementarity, failedResidualDetails.complementarity);
  EXPECT_EQ(
      retainedResidual.worstPrimalContact,
      failedResidualDetails.worstPrimalContact);
  EXPECT_EQ(
      retainedResidual.worstDualContact,
      failedResidualDetails.worstDualContact);
  EXPECT_EQ(
      retainedResidual.worstComplementarityContact,
      failedResidualDetails.worstComplementarityContact);
  EXPECT_EQ(solver.getLastFailedExactCoulombBestResidual(), failedBestResidual);
  EXPECT_EQ(
      solver.getLastFailedExactCoulombBestIteration(), failedBestIteration);
  EXPECT_EQ(solver.getLastFailedExactCoulombIterations(), failedIterations);
  EXPECT_EQ(solver.getLastFailedExactCoulombStepSize(), failedStepSize);
  EXPECT_EQ(solver.getLastFailedExactCoulombSafeStepSize(), failedSafeStepSize);
  EXPECT_EQ(
      solver.getLastFailedExactCoulombCouplingVariationRatio(),
      failedCouplingVariation);
  EXPECT_EQ(
      solver.getLastFailedExactCoulombShrinkIterations(),
      failedShrinkIterations);
}

TEST(ExactCoulombFbfConstraintSolver, DenseResidualPolishCanRecoverFailedSolve)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 0;
  options.fallbackToBoxedLcp = false;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = true;
  options.seedNormalImpulseFromDiagonal = false;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_EQ(
      solver.getLastExactCoulombFbfStatus(),
      dart::math::detail::ExactCoulombFbfStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombDenseResidualPolishUsed());
  EXPECT_FALSE(solver.getLastExactCoulombProjectedGradientRetryUsed());
  EXPECT_EQ(solver.getNumExactCoulombDenseResidualPolishes(), 1u);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 1u);
  EXPECT_EQ(solver.getNumExactCoulombFailures(), 0u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_LE(solver.getLastExactCoulombResidual(), options.tolerance);
  EXPECT_NEAR(contact->getAppliedImpulse()[0], 1.0, 1e-8);
  EXPECT_NEAR(contact->getAppliedImpulse()[1], 0.0, 1e-12);
  EXPECT_NEAR(contact->getAppliedImpulse()[2], 0.0, 1e-12);
}

TEST(ExactCoulombFbfConstraintSolver, FallsBackForUnsupportedGroup)
{
  auto scalar = std::make_shared<ScalarConstraint>();
  auto group = makeGroup(scalar);

  ExposedExactCoulombFbfConstraintSolver solver;
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::
          BoxedLcpFallback);
  EXPECT_EQ(
      solver.getLastExactCoulombBuildStatus(),
      dart::constraint::detail::ExactCoulombConstraintBuildStatus::
          UnsupportedDimension);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombAttempts(), 1u);
  EXPECT_EQ(solver.getNumExactCoulombMaxIterationsAccepted(), 0u);
  EXPECT_TRUE(std::isnan(solver.getWorstExactCoulombResidual()));
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 1u);
  EXPECT_TRUE(scalar->wasApplied());
  EXPECT_TRUE(std::isfinite(scalar->getAppliedImpulse()));
}

TEST(ExactCoulombFbfConstraintSolver, CanDisableFallback)
{
  auto scalar = std::make_shared<ScalarConstraint>();
  auto group = makeGroup(scalar);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.fallbackToBoxedLcp = false;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::
          UnsupportedProblem);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 0u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_FALSE(scalar->wasApplied());
}

TEST(ExactCoulombFbfConstraintSolver, RejectsInvalidStepSizeScale)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.fallbackToBoxedLcp = false;
  options.stepSizeScale = 0.0;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::InvalidOptions);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 0u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombFailures(), 1u);
  EXPECT_NEAR(contact->getAppliedImpulse().norm(), 0.0, 1e-12);
}

TEST(ExactCoulombFbfConstraintSolver, RejectsInvalidOuterRelaxation)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.fallbackToBoxedLcp = false;
  options.outerRelaxation = 0.0;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::InvalidOptions);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 0u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombFailures(), 1u);
  EXPECT_NEAR(contact->getAppliedImpulse().norm(), 0.0, 1e-12);
}

TEST(ExactCoulombFbfConstraintSolver, MapsAndValidatesLocalBlockSolverModes)
{
  using LocalSolver = dart::constraint::ExactCoulombFbfLocalBlockSolver;
  EXPECT_EQ(
      dart::constraint::ExactCoulombFbfConstraintSolverOptions{}
          .innerLocalSolver,
      LocalSolver::ExactMetricProjection);
  const std::array<LocalSolver, 3> localSolvers{
      LocalSolver::InverseEuclideanProjection,
      LocalSolver::ExactMetricProjection,
      LocalSolver::ProjectedGradient};

  for (const LocalSolver localSolver : localSolvers) {
    const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
    const auto activeRow = std::make_shared<Eigen::Index>(-1);
    auto contact = std::make_shared<ContactLikeConstraint>(
        0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
    auto group = makeGroup(contact);

    dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
    options.initialStepSize = 1.0;
    options.innerLocalSolver = localSolver;
    options.innerLocalIterations
        = localSolver == LocalSolver::ProjectedGradient ? 8 : 0;
    options.seedNormalImpulseFromDiagonal = false;
    options.enableProjectedGradientRetry = false;
    options.enableDenseResidualPolish = false;
    options.fallbackToBoxedLcp = false;

    ExposedExactCoulombFbfConstraintSolver solver(options);
    solver.setTimeStep(0.001);
    solver.solveConstrainedGroup(group);
    EXPECT_EQ(
        solver.getLastExactCoulombStatus(),
        dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  }

  dart::constraint::ExactCoulombFbfConstraintSolverOptions invalidOptions;
  invalidOptions.innerLocalSolver = static_cast<LocalSolver>(-1);
  invalidOptions.fallbackToBoxedLcp = false;
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0,
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      0.5,
      0.5,
      activeRow);
  auto group = makeGroup(contact);
  ExposedExactCoulombFbfConstraintSolver invalidSolver(invalidOptions);
  invalidSolver.setTimeStep(0.001);
  invalidSolver.solveConstrainedGroup(group);
  EXPECT_EQ(
      invalidSolver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::InvalidOptions);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    PersistentStepSizeGrowsAcrossStableSolvesAndRespectsFreshSafeCap)
{
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0,
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      0.5,
      0.5,
      activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.stepSizeRecoveryGrowthFactor = 1.25;
  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);

  solver.solveConstrainedGroup(group);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  ASSERT_FALSE(solver.getLastExactCoulombPersistentStepSizeUsed());
  const double firstStepSize = solver.getLastExactCoulombStepSize();
  ASSERT_TRUE(std::isfinite(firstStepSize));

  solver.solveConstrainedGroup(group);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombPersistentStepSizeUsed());
  EXPECT_NEAR(
      solver.getLastExactCoulombPersistentStepSizeRequest(),
      1.25 * firstStepSize,
      1e-12);
  EXPECT_NEAR(solver.getLastExactCoulombStepSize(), firstStepSize, 1e-12);
  EXPECT_NEAR(
      solver.getLastExactCoulombStepSize(),
      solver.getLastExactCoulombSafeStepSize(),
      1e-12);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    PersistedGammaCapsAtRawSafeStepWhenAutomaticScaleExceedsOne)
{
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0,
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      0.5,
      0.5,
      activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.stepSizeScale = 2.0;
  options.stepSizeRecoveryGrowthFactor = 1.25;
  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);

  solver.solveConstrainedGroup(group);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  ASSERT_FALSE(solver.getLastExactCoulombPersistentStepSizeUsed());
  const double automaticStepSize = solver.getLastExactCoulombStepSize();
  const double safeStepSize = solver.getLastExactCoulombSafeStepSize();
  ASSERT_TRUE(std::isfinite(automaticStepSize));
  ASSERT_TRUE(std::isfinite(safeStepSize));
  EXPECT_NEAR(automaticStepSize, 2.0 * safeStepSize, 1e-12);

  solver.solveConstrainedGroup(group);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombPersistentStepSizeUsed());
  EXPECT_NEAR(
      solver.getLastExactCoulombPersistentStepSizeRequest(),
      1.25 * automaticStepSize,
      1e-12);
  EXPECT_NEAR(solver.getLastExactCoulombStepSize(), safeStepSize, 1e-12);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    CrossStepPolicyUsesScaledSafeBoundAndNaturalResidualWarmCap)
{
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0,
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      0.5,
      0.5,
      activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.stepSizeScale = 10.0;
  options.stepSizeRecoveryGrowthFactor = 1.0 / 0.7;
  options.seedNormalImpulseFromDiagonal = false;
  ExposedExactCoulombFbfConstraintSolver solver(options);
  dart::constraint::ExactCoulombFbfCrossStepPolicyOptions policy;
  policy.persistentStepSizeSafeBoundScale = 10.0;
  policy.minimumStepSize = 1e-6;
  policy.maximumStepSize = 1e6;
  policy.warmStartResidualThreshold = 1e-4;
  policy.warmStartStepSizeCap = 0.1;
  policy.persistUncappedStepSizeOnWarmStartCap = true;
  solver.setExactCoulombCrossStepPolicyOptions(policy);
  solver.setTimeStep(0.001);

  solver.solveConstrainedGroup(group);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  const double firstStepSize = solver.getLastExactCoulombStepSize();
  const double safeStepSize = solver.getLastExactCoulombSafeStepSize();
  EXPECT_NEAR(firstStepSize, 10.0 * safeStepSize, 1e-12);
  EXPECT_FALSE(solver.getLastExactCoulombWarmStartStepSizeCapApplied());
  EXPECT_GT(solver.getLastExactCoulombInitialNaturalMapResidual(), 1e-4);

  solver.solveConstrainedGroup(group);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombPersistentStepSizeUsed());
  EXPECT_GT(
      solver.getLastExactCoulombPersistentStepSizeRequest(), firstStepSize);
  EXPECT_TRUE(solver.getLastExactCoulombWarmStartStepSizeCapApplied());
  EXPECT_NEAR(
      solver.getLastExactCoulombUncappedInitialStepSize(),
      firstStepSize,
      1e-12);
  EXPECT_NEAR(
      solver.getLastExactCoulombStepSize(), policy.warmStartStepSizeCap, 1e-12);
  EXPECT_LT(
      solver.getLastExactCoulombInitialNaturalMapResidual(),
      policy.warmStartResidualThreshold);
  EXPECT_EQ(
      solver.getLastExactCoulombNaturalMapResidual(),
      solver.getLastExactCoulombInitialNaturalMapResidual());
  EXPECT_EQ(solver.getNumExactCoulombWarmStartStepSizeCaps(), 1u);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    CrossStepPolicySkipsUnimprovedUnconvergedReactionCache)
{
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0,
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      0.5,
      0.5,
      activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  ExposedExactCoulombFbfConstraintSolver solver(options);
  dart::constraint::ExactCoulombFbfCrossStepPolicyOptions policy;
  policy.requireResidualImprovementForUnconvergedCacheSave = true;
  solver.setExactCoulombCrossStepPolicyOptions(policy);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);

  contact->setRhs(Eigen::Vector3d(0.2, 1.0, 0.0));
  options.maxOuterIterations = 0;
  options.acceptOuterMaxIterations = true;
  options.tolerance = 0.0;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.fallbackToBoxedLcp = false;
  solver.setExactCoulombOptions(options);
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::
          MaxIterationsAccepted);
  EXPECT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_EQ(
      solver.getLastExactCoulombNaturalMapResidual(),
      solver.getLastExactCoulombInitialNaturalMapResidual());
  EXPECT_EQ(solver.getNumExactCoulombUnconvergedCacheSkips(), 1u);
}

TEST(ExactCoulombFbfConstraintSolver, ValidatesAndCopiesCrossStepPolicy)
{
  using MatchMode = dart::constraint::ExactCoulombFbfWarmStartMatchMode;
  ExposedExactCoulombFbfConstraintSolver source;
  dart::constraint::ExactCoulombFbfCrossStepPolicyOptions policy;
  policy.warmStartMatchMode = MatchMode::OrderedBodyBLocalFeature;
  policy.warmStartNormalCosine = 0.95;
  policy.useStrictWarmStartMatchDistance = true;
  policy.warmStartMaxAge = 3;
  policy.persistentStepSizeSafeBoundScale = 10.0;
  policy.minimumStepSize = 1e-6;
  policy.maximumStepSize = 1e6;
  policy.warmStartResidualThreshold = 1e-4;
  policy.warmStartStepSizeCap = 1e4;
  policy.persistUncappedStepSizeOnWarmStartCap = true;
  policy.requireResidualImprovementForUnconvergedCacheSave = true;
  source.setExactCoulombCrossStepPolicyOptions(policy);

  const auto actual = source.getExactCoulombCrossStepPolicyOptions();
  EXPECT_EQ(actual.warmStartMatchMode, policy.warmStartMatchMode);
  EXPECT_EQ(actual.warmStartNormalCosine, policy.warmStartNormalCosine);
  EXPECT_EQ(actual.warmStartMaxAge, policy.warmStartMaxAge);
  EXPECT_EQ(actual.warmStartStepSizeCap, policy.warmStartStepSizeCap);

  ExposedExactCoulombFbfConstraintSolver copy;
  copy.setFromOtherConstraintSolver(source);
  const auto copied = copy.getExactCoulombCrossStepPolicyOptions();
  EXPECT_EQ(copied.warmStartMatchMode, policy.warmStartMatchMode);
  EXPECT_EQ(copied.persistentStepSizeSafeBoundScale, 10.0);
  EXPECT_TRUE(copied.persistUncappedStepSizeOnWarmStartCap);
  EXPECT_EQ(copy.getNumExactCoulombWarmStartStepSizeCaps(), 0u);

  auto invalid = policy;
  invalid.warmStartNormalCosine = 2.0;
  copy.setExactCoulombCrossStepPolicyOptions(invalid);
  EXPECT_EQ(
      copy.getExactCoulombCrossStepPolicyOptions().warmStartNormalCosine,
      policy.warmStartNormalCosine);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    PostCorrectionProjectionDefaultsOnAndCopies)
{
  ExposedExactCoulombFbfConstraintSolver source;
  EXPECT_TRUE(source.getExactCoulombPostCorrectionProjectionEnabled());

  source.setExactCoulombPostCorrectionProjectionEnabled(false);
  EXPECT_FALSE(source.getExactCoulombPostCorrectionProjectionEnabled());

  ExposedExactCoulombFbfConstraintSolver copy;
  copy.setFromOtherConstraintSolver(source);
  EXPECT_FALSE(copy.getExactCoulombPostCorrectionProjectionEnabled());

  copy.setExactCoulombPostCorrectionProjectionEnabled(true);
  EXPECT_TRUE(copy.getExactCoulombPostCorrectionProjectionEnabled());
}

TEST(
    ExactCoulombFbfConstraintSolver,
    SourceInnerInitializationDefaultsOffCopiesAndResets)
{
  ExposedExactCoulombFbfConstraintSolver source;
  EXPECT_FALSE(source.getExactCoulombSourceInnerInitializationEnabled());

  source.setExactCoulombSourceInnerInitializationEnabled(true);
  EXPECT_TRUE(source.getExactCoulombSourceInnerInitializationEnabled());

  ExposedExactCoulombFbfConstraintSolver copy;
  copy.setFromOtherConstraintSolver(source);
  EXPECT_TRUE(copy.getExactCoulombSourceInnerInitializationEnabled());

  dart::constraint::BoxedLcpConstraintSolver boxed;
  copy.setFromOtherConstraintSolver(boxed);
  EXPECT_FALSE(copy.getExactCoulombSourceInnerInitializationEnabled());
}

TEST(
    ExactCoulombFbfConstraintSolver,
    ChangingSourceInnerInitializationInvalidatesCrossStepState)
{
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0,
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      0.5,
      0.5,
      activeRow);
  auto group = makeGroup(contact);

  ExposedExactCoulombFbfConstraintSolver solver;
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);
  solver.solveConstrainedGroup(group);
  ASSERT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  ASSERT_TRUE(solver.getLastExactCoulombPersistentStepSizeUsed());

  solver.setExactCoulombSourceInnerInitializationEnabled(true);
  solver.solveConstrainedGroup(group);
  EXPECT_FALSE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_FALSE(solver.getLastExactCoulombPersistentStepSizeUsed());
}

TEST(
    ExactCoulombFbfConstraintSolver,
    PersistentStepSizeHoldsAfterAnyRejectedTrial)
{
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0,
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1.0, 1.0, 0.0),
      0.5,
      0.5,
      activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 1;
  options.acceptOuterMaxIterations = true;
  options.tolerance = 0.0;
  options.couplingVariationTolerance = 0.05;
  options.maxStepShrinkIterations = 20;
  options.stepSizeRecoveryGrowthFactor = 2.0;
  options.seedNormalImpulseFromDiagonal = false;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.fallbackToBoxedLcp = false;

  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);
  ASSERT_GT(solver.getLastExactCoulombShrinkIterations(), 0);
  const double shrunkenStepSize = solver.getLastExactCoulombStepSize();

  solver.solveConstrainedGroup(group);
  EXPECT_TRUE(solver.getLastExactCoulombPersistentStepSizeUsed());
  EXPECT_NEAR(
      solver.getLastExactCoulombPersistentStepSizeRequest(),
      shrunkenStepSize,
      1e-12);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    PersistentStepSizeDoesNotCrossGroupsAndOptionsResetIt)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto first = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto second = std::make_shared<ContactLikeConstraint>(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);
  auto firstGroup = makeGroup(first);
  auto secondGroup = makeGroup(second);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(firstGroup);
  solver.solveConstrainedGroup(secondGroup);
  EXPECT_FALSE(solver.getLastExactCoulombPersistentStepSizeUsed());

  solver.solveConstrainedGroup(secondGroup);
  ASSERT_TRUE(solver.getLastExactCoulombPersistentStepSizeUsed());
  solver.setExactCoulombOptions(options);
  solver.solveConstrainedGroup(secondGroup);
  EXPECT_FALSE(solver.getLastExactCoulombPersistentStepSizeUsed());
}

TEST(
    ExactCoulombFbfConstraintSolver,
    PersistentStepSizeNeverOverridesExplicitOrFixedGammaOptions)
{
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0,
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      0.5,
      0.5,
      activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions explicitOptions;
  explicitOptions.initialStepSize = 0.25;
  ExposedExactCoulombFbfConstraintSolver explicitSolver(explicitOptions);
  explicitSolver.setTimeStep(0.001);
  explicitSolver.solveConstrainedGroup(group);
  explicitSolver.solveConstrainedGroup(group);
  EXPECT_FALSE(explicitSolver.getLastExactCoulombPersistentStepSizeUsed());
  EXPECT_NEAR(explicitSolver.getLastExactCoulombStepSize(), 0.25, 1e-12);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions fixedOptions;
  fixedOptions.enableAdaptiveStepSize = false;
  ExposedExactCoulombFbfConstraintSolver fixedSolver(fixedOptions);
  fixedSolver.setTimeStep(0.001);
  fixedSolver.solveConstrainedGroup(group);
  fixedSolver.solveConstrainedGroup(group);
  EXPECT_FALSE(fixedSolver.getLastExactCoulombPersistentStepSizeUsed());
}

TEST(
    ExactCoulombFbfConstraintSolver,
    FailedPersistentGammaRetriesAutomaticPolicyBeforeFallback)
{
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0,
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      0.5,
      0.5,
      activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 13;
  options.tolerance = 1e-6;
  options.stepSizeScale = 2.0;
  options.seedNormalImpulseFromDiagonal = false;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.fallbackToBoxedLcp = false;
  ExposedExactCoulombFbfConstraintSolver solver(options);
  solver.setTimeStep(0.001);

  solver.solveConstrainedGroup(group);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  ASSERT_EQ(solver.getNumExactCoulombPersistentStepSizeRetries(), 0u);
  const std::size_t firstTotalIterations
      = solver.getTotalExactCoulombIterations();

  // Reuse the same group with a changed free velocity. The raw-safe persisted
  // gamma exhausts this deliberately small budget, while the current step's
  // normal automatic 2x-safe policy converges within it.
  contact->setRhs(Eigen::Vector3d(0.2, 1.0, 0.0));
  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);
  EXPECT_TRUE(solver.getLastExactCoulombPersistentStepSizeUsed());
  EXPECT_EQ(solver.getNumExactCoulombPersistentStepSizeRetries(), 1u);
  EXPECT_EQ(solver.getNumExactCoulombFailures(), 0u);
  EXPECT_EQ(solver.getNumBoxedLcpFallbacks(), 0u);
  EXPECT_FALSE(solver.getLastExactCoulombProjectedGradientRetryUsed());
  EXPECT_LE(solver.getLastExactCoulombResidual(), options.tolerance);
  EXPECT_NEAR(
      solver.getLastExactCoulombStepSize(),
      options.stepSizeScale * solver.getLastExactCoulombSafeStepSize(),
      1e-12);
  EXPECT_GT(
      solver.getTotalExactCoulombIterations() - firstTotalIterations,
      static_cast<std::size_t>(solver.getLastExactCoulombIterations()));

  // A recovery invalidates only the failed gamma hint. The successful
  // reaction remains a warm start, but the next solve must choose gamma fresh.
  solver.solveConstrainedGroup(group);
  EXPECT_FALSE(solver.getLastExactCoulombPersistentStepSizeUsed());
  EXPECT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_EQ(solver.getNumExactCoulombPersistentStepSizeRetries(), 1u);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    SourceContinuationDefaultsOffValidatesCopiesAndCleansLifecycleState)
{
  using Solver = dart::constraint::ExactCoulombFbfConstraintSolver;
  using Status = dart::constraint::ExactCoulombFbfConstraintSolverStatus;
  EXPECT_EQ(static_cast<int>(Status::NotRun), 0);
  EXPECT_EQ(static_cast<int>(Status::Success), 1);
  EXPECT_EQ(static_cast<int>(Status::MaxIterationsAccepted), 2);
  EXPECT_EQ(static_cast<int>(Status::InvalidOptions), 3);
  EXPECT_EQ(static_cast<int>(Status::UnsupportedProblem), 4);
  EXPECT_EQ(static_cast<int>(Status::FbfFailed), 5);
  EXPECT_EQ(static_cast<int>(Status::BoxedLcpFallback), 6);
  EXPECT_EQ(static_cast<int>(Status::PlateauAccepted), 7);

  ExposedExactCoulombFbfConstraintSolver source;
  EXPECT_FALSE(source.getExactCoulombSourceContinuationOptions().enabled);
  EXPECT_FALSE(source.getLastExactCoulombSourceContinuationActive());
  EXPECT_EQ(source.getNumExactCoulombPlateausAccepted(), 0u);
  EXPECT_EQ(source.getNumExactCoulombLineSearchShrinkCaps(), 0u);

  dart::constraint::ExactCoulombFbfSourceContinuationOptions policy;
  policy.enabled = true;
  policy.residualCheckInterval = 3;
  policy.plateauPatience = 2;
  policy.plateauRelativeTolerance = 0.02;
  policy.stepSizeBacktrackLimit = 4;
  policy.couplingVariationSkipThreshold = 2e-10;
  source.setExactCoulombSourceContinuationOptions(policy);
  dart::constraint::ExactCoulombFbfCrossStepPolicyOptions crossStepPolicy;
  crossStepPolicy.warmStartMaxAge = 3;
  source.setExactCoulombCrossStepPolicyOptions(crossStepPolicy);
  EXPECT_TRUE(source.getExactCoulombSourceContinuationOptions().enabled);
  EXPECT_EQ(
      source.getExactCoulombSourceContinuationOptions().plateauPatience, 2);

  ExposedExactCoulombFbfConstraintSolver copy;
  copy.setFromOtherConstraintSolver(source);
  const auto copied = copy.getExactCoulombSourceContinuationOptions();
  EXPECT_TRUE(copied.enabled);
  EXPECT_EQ(copied.residualCheckInterval, 3);
  EXPECT_EQ(copied.plateauPatience, 2);
  EXPECT_EQ(copied.plateauRelativeTolerance, 0.02);
  EXPECT_EQ(copied.stepSizeBacktrackLimit, 4);
  EXPECT_EQ(copied.couplingVariationSkipThreshold, 2e-10);
  EXPECT_EQ(copy.getExactCoulombCrossStepPolicyOptions().warmStartMaxAge, 3);

  auto invalid = policy;
  invalid.residualCheckInterval = 0;
  copy.setExactCoulombSourceContinuationOptions(invalid);
  EXPECT_EQ(
      copy.getExactCoulombSourceContinuationOptions().residualCheckInterval,
      policy.residualCheckInterval);

  dart::constraint::BoxedLcpConstraintSolver boxed;
  copy.setFromOtherConstraintSolver(boxed);
  EXPECT_FALSE(copy.getExactCoulombSourceContinuationOptions().enabled);

  alignas(Solver) unsigned char storage[sizeof(Solver)];
  auto* first = new (storage) Solver;
  first->setExactCoulombSourceContinuationOptions(policy);
  ASSERT_TRUE(first->getExactCoulombSourceContinuationOptions().enabled);
  first->~Solver();
  auto* second = new (storage) Solver;
  EXPECT_FALSE(second->getExactCoulombSourceContinuationOptions().enabled);
  EXPECT_FALSE(second->getLastExactCoulombSourceContinuationActive());
  EXPECT_EQ(second->getNumExactCoulombPlateausAccepted(), 0u);
  EXPECT_EQ(second->getNumExactCoulombLineSearchShrinkCaps(), 0u);
  second->~Solver();
}

TEST(
    ExactCoulombFbfConstraintSolver,
    SourceContinuationAcceptsFiniteMaxAndRecordsPerGroupOutcome)
{
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0,
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      0.5,
      0.5,
      activeRow);
  auto group = makeGroup(contact);

  auto options = makeZeroBudgetWarmStartOptions();
  options.maxResidualHistorySamples = 4;
  options.maxResidualHistoryRecords = 2;
  ExposedExactCoulombFbfConstraintSolver solver(options);
  dart::constraint::ExactCoulombFbfSourceContinuationOptions sourcePolicy;
  sourcePolicy.enabled = true;
  sourcePolicy.plateauPatience = 0;
  solver.setExactCoulombSourceContinuationOptions(sourcePolicy);
  solver.setTimeStep(0.001);

  solver.solveConstrainedGroup(group);

  EXPECT_TRUE(solver.getExactCoulombSourceContinuationOptions().enabled);
  EXPECT_EQ(
      solver.getExactCoulombSourceContinuationOptions().plateauPatience, 0);
  EXPECT_TRUE(solver.getLastExactCoulombSourceContinuationActive());
  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::
          MaxIterationsAccepted);
  EXPECT_EQ(
      solver.getLastExactCoulombFbfStatus(),
      dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  EXPECT_EQ(solver.getNumExactCoulombMaxIterationsAccepted(), 1u);
  EXPECT_EQ(solver.getNumExactCoulombPlateausAccepted(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombFailures(), 0u);
  EXPECT_FALSE(solver.getLastExactCoulombLineSearchShrinkCapReached());
  EXPECT_EQ(solver.getLastExactCoulombLineSearchShrinkCapCount(), 0);
  EXPECT_TRUE(std::isnan(solver.getLastExactCoulombInnerSolveStepSize()));

  const auto& records = solver.getExactCoulombResidualHistoryRecords();
  ASSERT_EQ(records.size(), 1u);
  EXPECT_TRUE(records[0].sourceContinuationActive);
  EXPECT_EQ(
      records[0].status,
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::
          MaxIterationsAccepted);
  EXPECT_EQ(
      records[0].fbfStatus,
      dart::math::detail::ExactCoulombFbfStatus::MaxIterations);
  EXPECT_EQ(records[0].finalResidual, solver.getLastExactCoulombResidual());
  EXPECT_EQ(
      records[0].finalNaturalMapResidual,
      solver.getLastExactCoulombNaturalMapResidual());
  EXPECT_EQ(records[0].shrinkIterations, 0);
  EXPECT_EQ(records[0].lineSearchShrinkCapCount, 0);
  EXPECT_TRUE(std::isnan(records[0].lastInnerSolveStepSize));
  EXPECT_TRUE(std::isnan(records[0].plateauReferenceNaturalMapResidual));
  EXPECT_TRUE(std::isnan(records[0].plateauRelativeImprovement));
}

TEST(
    ExactCoulombFbfConstraintSolver,
    SourceContinuationRequestedButInactiveForUnsupportedGroup)
{
  auto scalar = std::make_shared<ScalarConstraint>();
  auto group = makeGroup(scalar);
  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.fallbackToBoxedLcp = false;
  ExposedExactCoulombFbfConstraintSolver solver(options);
  dart::constraint::ExactCoulombFbfSourceContinuationOptions sourcePolicy;
  sourcePolicy.enabled = true;
  solver.setExactCoulombSourceContinuationOptions(sourcePolicy);
  solver.setTimeStep(0.001);

  solver.solveConstrainedGroup(group);

  EXPECT_TRUE(solver.getExactCoulombSourceContinuationOptions().enabled);
  EXPECT_FALSE(solver.getLastExactCoulombSourceContinuationActive());
  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::
          UnsupportedProblem);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombFailures(), 1u);
}

TEST(
    ExactCoulombFbfConstraintSolver,
    SourcePolicyChangeInvalidatesCacheAndSkippedReactionStillPersistsGamma)
{
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0,
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      0.5,
      0.5,
      activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.seedNormalImpulseFromDiagonal = false;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.fallbackToBoxedLcp = false;
  ExposedExactCoulombFbfConstraintSolver solver(options);
  dart::constraint::ExactCoulombFbfCrossStepPolicyOptions crossStepPolicy;
  crossStepPolicy.requireResidualImprovementForUnconvergedCacheSave = true;
  solver.setExactCoulombCrossStepPolicyOptions(crossStepPolicy);
  solver.setTimeStep(0.001);
  solver.solveConstrainedGroup(group);
  solver.solveConstrainedGroup(group);
  ASSERT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  ASSERT_TRUE(solver.getLastExactCoulombPersistentStepSizeUsed());

  dart::constraint::ExactCoulombFbfSourceContinuationOptions sourcePolicy;
  sourcePolicy.enabled = true;
  sourcePolicy.residualCheckInterval = 1;
  solver.setExactCoulombSourceContinuationOptions(sourcePolicy);
  solver.solveConstrainedGroup(group);
  ASSERT_FALSE(solver.getLastExactCoulombWarmStartUsed());
  ASSERT_FALSE(solver.getLastExactCoulombPersistentStepSizeUsed());
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::Success);

  contact->setRhs(Eigen::Vector3d(0.2, 1.0, 0.0));
  options.maxOuterIterations = 0;
  options.tolerance = 0.0;
  solver.setExactCoulombOptions(options);
  solver.solveConstrainedGroup(group);
  ASSERT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::
          MaxIterationsAccepted);
  EXPECT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_EQ(solver.getNumExactCoulombUnconvergedCacheSkips(), 1u);

  solver.solveConstrainedGroup(group);
  EXPECT_TRUE(solver.getLastExactCoulombWarmStartUsed());
  EXPECT_TRUE(solver.getLastExactCoulombPersistentStepSizeUsed());
  EXPECT_EQ(solver.getNumExactCoulombUnconvergedCacheSkips(), 2u);
}

TEST(
    ExactCoulombFbfConstraintSolver, SourceContinuationAcceptsPlateauDistinctly)
{
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0,
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1.0, 1.0, 0.0),
      0.5,
      0.5,
      activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.maxOuterIterations = 10;
  options.tolerance = 0.0;
  options.seedNormalImpulseFromDiagonal = false;
  options.enableProjectedGradientRetry = false;
  options.enableDenseResidualPolish = false;
  options.fallbackToBoxedLcp = false;
  options.maxResidualHistorySamples = 16;
  options.maxResidualHistoryRecords = 1;
  ExposedExactCoulombFbfConstraintSolver solver(options);
  dart::constraint::ExactCoulombFbfSourceContinuationOptions sourcePolicy;
  sourcePolicy.enabled = true;
  sourcePolicy.residualCheckInterval = 1;
  sourcePolicy.plateauPatience = 1;
  sourcePolicy.plateauRelativeTolerance = std::numeric_limits<double>::max();
  solver.setExactCoulombSourceContinuationOptions(sourcePolicy);
  solver.setTimeStep(0.001);

  solver.solveConstrainedGroup(group);

  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::PlateauAccepted);
  EXPECT_EQ(
      solver.getLastExactCoulombFbfStatus(),
      dart::math::detail::ExactCoulombFbfStatus::Plateau);
  EXPECT_EQ(solver.getLastExactCoulombIterations(), 2);
  EXPECT_EQ(solver.getNumExactCoulombPlateausAccepted(), 1u);
  EXPECT_EQ(solver.getNumExactCoulombMaxIterationsAccepted(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombFailures(), 0u);

  const auto& records = solver.getExactCoulombResidualHistoryRecords();
  ASSERT_EQ(records.size(), 1u);
  EXPECT_EQ(
      records[0].status,
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::PlateauAccepted);
  EXPECT_TRUE(std::isfinite(records[0].plateauReferenceNaturalMapResidual));
  EXPECT_GT(records[0].plateauReferenceNaturalMapResidual, 0.0);
  EXPECT_TRUE(std::isfinite(records[0].plateauRelativeImprovement));
  EXPECT_NEAR(
      records[0].plateauRelativeImprovement,
      (records[0].plateauReferenceNaturalMapResidual
       - records[0].finalNaturalMapResidual)
          / records[0].plateauReferenceNaturalMapResidual,
      1e-15);
  EXPECT_LT(
      records[0].plateauRelativeImprovement,
      sourcePolicy.plateauRelativeTolerance);
}

TEST(ExactCoulombFbfConstraintSolver, SourceContinuationKeepsInnerFailureFatal)
{
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  auto contact = std::make_shared<ContactLikeConstraint>(
      0,
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      0.5,
      0.5,
      activeRow);
  auto group = makeGroup(contact);

  dart::constraint::ExactCoulombFbfConstraintSolverOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;
  options.innerMaxSweeps = 1;
  options.innerLocalIterations = 1;
  options.innerTolerance = 0.0;
  options.acceptInnerMaxIterations = false;
  options.seedNormalImpulseFromDiagonal = false;
  options.enableProjectedGradientRetry = true;
  options.enableDenseResidualPolish = true;
  options.fallbackToBoxedLcp = false;
  ExposedExactCoulombFbfConstraintSolver solver(options);
  dart::constraint::ExactCoulombFbfSourceContinuationOptions sourcePolicy;
  sourcePolicy.enabled = true;
  solver.setExactCoulombSourceContinuationOptions(sourcePolicy);
  solver.setTimeStep(0.001);

  solver.solveConstrainedGroup(group);

  EXPECT_TRUE(solver.getLastExactCoulombSourceContinuationActive());
  EXPECT_EQ(
      solver.getLastExactCoulombStatus(),
      dart::constraint::ExactCoulombFbfConstraintSolverStatus::FbfFailed);
  EXPECT_EQ(
      solver.getLastExactCoulombFbfStatus(),
      dart::math::detail::ExactCoulombFbfStatus::InnerSolverFailed);
  EXPECT_EQ(solver.getNumExactCoulombSolves(), 0u);
  EXPECT_EQ(solver.getNumExactCoulombFailures(), 1u);
  EXPECT_FALSE(solver.getLastExactCoulombProjectedGradientRetryUsed());
  EXPECT_FALSE(solver.getLastExactCoulombDenseResidualPolishUsed());
}
