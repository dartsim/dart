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

#include <dart/constraint/detail/ExactCoulombConstraintAdapter.hpp>

#include <dart/math/detail/ExactCoulombFbfSolver.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>

namespace {

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

  void getVelocityChange(double* velocityChange, bool withCfm) override
  {
    Eigen::Map<Eigen::Vector3d> velocity(velocityChange);
    velocity.setZero();
    if (*mActiveRow < 0) {
      return;
    }

    velocity = mDelassus.block<1, 3>(*mActiveRow, mOffset).transpose();
    if (withCfm && *mActiveRow >= mOffset && *mActiveRow < mOffset + 3) {
      velocity[*mActiveRow - mOffset] += 0.125;
    }
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

private:
  Eigen::Index mOffset;
  Eigen::MatrixXd mDelassus;
  Eigen::Vector3d mRhs;
  double mPrimaryCoefficient;
  double mSecondaryCoefficient;
  std::shared_ptr<Eigen::Index> mActiveRow;
  Eigen::Vector3d mAppliedImpulse;
};

class OneDimensionalConstraint final : public dart::constraint::ConstraintBase
{
public:
  OneDimensionalConstraint()
  {
    mDim = 1u;
  }

  void update() override {}

  void getInformation(dart::constraint::ConstraintInfo*) override {}

  void applyUnitImpulse(std::size_t) override {}

  void getVelocityChange(double*, bool) override {}

  void excite() override {}

  void unexcite() override {}

  void applyImpulse(double*) override {}

  bool isActive() const override
  {
    return true;
  }

  dart::dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return nullptr;
  }
};

auto makeDenseDelassusOperator(Eigen::MatrixXd delassus)
{
  return [delassus](
             const Eigen::Ref<const Eigen::VectorXd>& input,
             Eigen::Ref<Eigen::VectorXd> output) {
    output.noalias() = delassus * input;
  };
}

void expectVectorNear(
    const Eigen::VectorXd& actual,
    const Eigen::VectorXd& expected,
    double tolerance)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (Eigen::Index i = 0; i < actual.size(); ++i) {
    EXPECT_NEAR(actual[i], expected[i], tolerance);
  }
}

} // namespace

TEST(ExactCoulombConstraintAdapter, BuildsProblemAndDelassusFromContactRows)
{
  Eigen::MatrixXd delassus = Eigen::MatrixXd::Zero(6, 6);
  delassus.diagonal() << 2.0, 2.5, 3.0, 4.0, 4.5, 5.0;
  delassus(0, 3) = 0.2;
  delassus(3, 0) = 0.2;
  delassus(1, 4) = -0.1;
  delassus(4, 1) = -0.1;
  delassus(2, 5) = 0.05;
  delassus(5, 2) = 0.05;

  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  ContactLikeConstraint first(
      0, delassus, Eigen::Vector3d(1.0, -0.2, 0.1), 0.5, 0.5, activeRow);
  ContactLikeConstraint second(
      3, delassus, Eigen::Vector3d(0.4, 0.3, -0.1), 0.8, 0.8, activeRow);

  auto result = dart::constraint::detail::buildExactCoulombConstraintProblem(
      {&first, &second});

  EXPECT_EQ(
      result.status,
      dart::constraint::detail::ExactCoulombConstraintBuildStatus::Success);

  Eigen::VectorXd expectedFreeVelocity(6);
  expectedFreeVelocity << -1.0, 0.2, -0.1, -0.4, -0.3, 0.1;
  expectVectorNear(
      result.contactProblem.freeVelocity, expectedFreeVelocity, 1e-12);
  expectVectorNear(
      result.contactProblem.coefficients, Eigen::Vector2d(0.5, 0.8), 1e-12);
  EXPECT_TRUE(result.delassus.isApprox(delassus, 1e-12));

  Eigen::VectorXd expectedInitialGuess(6);
  expectedInitialGuess << 0.49246231155778897, -0.07740213523131673,
      0.033672278713118856, 0.079159658663414983, 0.060434201221917394,
      -0.018923750180187767;
  expectVectorNear(result.initialGuess, expectedInitialGuess, 1e-12);

  Eigen::VectorXd input(6);
  input << 1.2, -0.3, 0.7, -0.2, 0.4, -1.1;
  Eigen::VectorXd matrixFreeProduct(6);
  ASSERT_TRUE(dart::constraint::detail::applyExactCoulombConstraintDelassus(
      result, input, matrixFreeProduct));
  expectVectorNear(matrixFreeProduct, delassus * input, 1e-12);

  dart::constraint::detail::ExactCoulombConstraintBuildOptions options;
  options.seedNormalImpulseFromDiagonal = false;
  result = dart::constraint::detail::buildExactCoulombConstraintProblem(
      {&first, &second}, options);
  ASSERT_EQ(
      result.status,
      dart::constraint::detail::ExactCoulombConstraintBuildStatus::Success);
  expectVectorNear(result.initialGuess, Eigen::VectorXd::Zero(6), 1e-12);

  ASSERT_TRUE(
      dart::constraint::detail::seedExactCoulombImpulseFromDelassusOperator(
          result, makeDenseDelassusOperator(delassus)));
  Eigen::VectorXd expectedOperatorSeed(6);
  expectedOperatorSeed << 0.5, -0.08, 1.0 / 30.0, 0.1, 1.0 / 15.0, -0.02;
  expectVectorNear(result.initialGuess, expectedOperatorSeed, 1e-12);
}

TEST(
    ExactCoulombConstraintAdapter,
    MatrixFreeDelassusOperatorMatchesRegularizedDenseSnapshot)
{
  Eigen::MatrixXd delassus = Eigen::MatrixXd::Zero(6, 6);
  delassus.diagonal() << 2.0, 2.5, 3.0, 4.0, 4.5, 5.0;
  delassus(0, 3) = 0.2;
  delassus(3, 0) = 0.2;
  delassus(1, 4) = -0.1;
  delassus(4, 1) = -0.1;
  delassus(2, 5) = 0.05;
  delassus(5, 2) = 0.05;

  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  ContactLikeConstraint first(
      0, delassus, Eigen::Vector3d(1.0, -0.2, 0.1), 0.5, 0.5, activeRow);
  ContactLikeConstraint second(
      3, delassus, Eigen::Vector3d(0.4, 0.3, -0.1), 0.8, 0.8, activeRow);

  dart::constraint::detail::ExactCoulombConstraintBuildOptions options;
  options.includeConstraintRegularization = true;
  const auto result
      = dart::constraint::detail::buildExactCoulombConstraintProblem(
          {&first, &second}, options);

  ASSERT_EQ(
      result.status,
      dart::constraint::detail::ExactCoulombConstraintBuildStatus::Success);

  Eigen::VectorXd input(6);
  input << -0.4, 0.1, 0.9, 1.3, -0.8, 0.2;
  Eigen::VectorXd matrixFreeProduct(6);
  ASSERT_TRUE(dart::constraint::detail::applyExactCoulombConstraintDelassus(
      result, input, matrixFreeProduct, true));
  expectVectorNear(matrixFreeProduct, result.delassus * input, 1e-12);
}

TEST(ExactCoulombConstraintAdapter, RejectsUnsupportedRows)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  ContactLikeConstraint anisotropic(
      0, delassus, Eigen::Vector3d::Zero(), 0.5, 0.7, activeRow);

  auto result = dart::constraint::detail::buildExactCoulombConstraintProblem(
      {&anisotropic});

  EXPECT_EQ(
      result.status,
      dart::constraint::detail::ExactCoulombConstraintBuildStatus::
          UnsupportedAnisotropicFriction);

  OneDimensionalConstraint scalarConstraint;
  result = dart::constraint::detail::buildExactCoulombConstraintProblem(
      {&scalarConstraint});

  EXPECT_EQ(
      result.status,
      dart::constraint::detail::ExactCoulombConstraintBuildStatus::
          UnsupportedDimension);
}

TEST(ExactCoulombConstraintAdapter, SolvesAndAppliesSingleContactFbfImpulse)
{
  const Eigen::Matrix3d delassus = Eigen::Matrix3d::Identity();
  const auto activeRow = std::make_shared<Eigen::Index>(-1);
  ContactLikeConstraint contact(
      0, delassus, Eigen::Vector3d(1.0, 0.0, 0.0), 0.5, 0.5, activeRow);

  auto problem = dart::constraint::detail::buildExactCoulombConstraintProblem(
      {&contact});
  ASSERT_EQ(
      problem.status,
      dart::constraint::detail::ExactCoulombConstraintBuildStatus::Success);

  const auto applyDelassus = makeDenseDelassusOperator(problem.delassus);

  dart::math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions
      innerOptions;
  innerOptions.maxSweeps = 20;
  innerOptions.localIterations = 4;
  innerOptions.tolerance = 1e-12;

  const auto innerSolver
      = [&applyDelassus, &innerOptions](
            const auto& innerProblem,
            const Eigen::Ref<const Eigen::VectorXd>& referenceReaction,
            const Eigen::Ref<const Eigen::VectorXd>& frozenCoupling,
            double stepSizeGamma,
            Eigen::Ref<Eigen::VectorXd> output) {
          const auto innerResult
              = dart::math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
                  innerProblem,
                  referenceReaction,
                  frozenCoupling,
                  stepSizeGamma,
                  applyDelassus,
                  innerOptions);
          output = innerResult.reaction;
          return innerResult.status
                 == dart::math::detail::ExactCoulombFrozenConeStatus::Success;
        };

  dart::math::detail::ExactCoulombFbfOptions options;
  options.initialStepSize = 1.0;
  options.maxOuterIterations = 80;
  options.tolerance = 1e-10;

  const auto solution = dart::math::detail::solveExactCoulombFbf(
      problem.contactProblem,
      problem.initialGuess,
      applyDelassus,
      innerSolver,
      options);

  ASSERT_EQ(
      solution.status, dart::math::detail::ExactCoulombFbfStatus::Success);
  ASSERT_TRUE(dart::constraint::detail::applyExactCoulombConstraintImpulses(
      problem, solution.reaction));
  EXPECT_NEAR(contact.getAppliedImpulse()[0], 1.0, 1e-9);
  EXPECT_NEAR(contact.getAppliedImpulse()[1], 0.0, 1e-12);
  EXPECT_NEAR(contact.getAppliedImpulse()[2], 0.0, 1e-12);
}
