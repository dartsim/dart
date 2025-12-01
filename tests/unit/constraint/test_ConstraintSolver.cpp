/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "helpers/GTestUtils.hpp"

#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/BoxedLcpSolver.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/constraint/ContactSurface.hpp"
#include "dart/simulation/World.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <vector>

using namespace dart;
using namespace dart::simulation;
using namespace dart::test;

namespace {

// Minimal constraint used to exercise the LCP pipeline.
class DummyConstraint : public constraint::ConstraintBase
{
public:
  DummyConstraint()
  {
    mDim = 1;
  }

  void update() override {}

  void getInformation(constraint::ConstraintInfo* info) override
  {
    // Populate trivial bounds and bias.
    info->lo[0] = 0.0;
    info->hi[0] = 1.0;
    info->b[0] = 0.0;
    info->w[0] = 0.0;
    info->findex[0] = -1;
  }

  void applyUnitImpulse(std::size_t /*index*/) override {}

  void getVelocityChange(double* vel, bool /*withCfm*/) override
  {
    vel[0] = 1.0;
  }

  void excite() override {}
  void unexcite() override {}

  void applyImpulse(double* lambda) override
  {
    lastAppliedImpulse = lambda[0];
  }

  bool isActive() const override
  {
    return true;
  }

  dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return nullptr;
  }

  double lastAppliedImpulse{std::numeric_limits<double>::quiet_NaN()};
};

// Minimal multi-dimensional constraint to verify partial NaN handling.
class MultiDummyConstraint : public constraint::ConstraintBase
{
public:
  explicit MultiDummyConstraint(std::size_t dim)
    : lastAppliedImpulse(dim, std::numeric_limits<double>::quiet_NaN())
  {
    mDim = dim;
  }

  void update() override {}

  void getInformation(constraint::ConstraintInfo* info) override
  {
    for (std::size_t i = 0; i < mDim; ++i) {
      info->lo[i] = 0.0;
      info->hi[i] = 1.0;
      info->b[i] = 0.0;
      info->w[i] = 0.0;
      info->findex[i] = -1;
    }
  }

  void applyUnitImpulse(std::size_t /*index*/) override {}

  void getVelocityChange(double* vel, bool /*withCfm*/) override
  {
    for (std::size_t i = 0; i < mDim; ++i)
      vel[i] = 1.0;
  }

  void excite() override {}
  void unexcite() override {}

  void applyImpulse(double* lambda) override
  {
    for (std::size_t i = 0; i < mDim; ++i)
      lastAppliedImpulse[i] = lambda[i];
  }

  bool isActive() const override
  {
    return true;
  }

  dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return nullptr;
  }

  std::vector<double> lastAppliedImpulse;
};

// Primary solver that reports success but writes NaN into x.
class NanBoxedLcpSolver : public constraint::BoxedLcpSolver
{
public:
  const std::string& getType() const override
  {
    static const std::string type{"NanBoxedLcpSolver"};
    return type;
  }

  dart::math::LcpResult solve(
      const Eigen::MatrixXd& A,
      const Eigen::VectorXd& /*b*/,
      const Eigen::VectorXd& /*lo*/,
      const Eigen::VectorXd& /*hi*/,
      const Eigen::VectorXi& /*findex*/,
      Eigen::VectorXd& x,
      const dart::math::LcpOptions& /*options*/) override
  {
    x = Eigen::VectorXd::Constant(
        A.rows(), std::numeric_limits<double>::quiet_NaN());
    dart::math::LcpResult res;
    res.status = dart::math::LcpSolverStatus::Success; // claims success despite
                                                       // NaN output
    return res;
  }

#if DART_BUILD_MODE_DEBUG
  bool canSolve(const Eigen::MatrixXd& /*A*/) override
  {
    return true;
  }
#endif
};

// Secondary solver that succeeds and writes a constant impulse.
class ConstantBoxedLcpSolver : public constraint::BoxedLcpSolver
{
public:
  explicit ConstantBoxedLcpSolver(double value) : mValue(value) {}

  const std::string& getType() const override
  {
    static const std::string type{"ConstantBoxedLcpSolver"};
    return type;
  }

  dart::math::LcpResult solve(
      const Eigen::MatrixXd& A,
      const Eigen::VectorXd& /*b*/,
      const Eigen::VectorXd& /*lo*/,
      const Eigen::VectorXd& /*hi*/,
      const Eigen::VectorXi& /*findex*/,
      Eigen::VectorXd& x,
      const dart::math::LcpOptions& /*options*/) override
  {
    x = Eigen::VectorXd::Constant(A.rows(), mValue);
    dart::math::LcpResult res;
    res.status = dart::math::LcpSolverStatus::Success;
    return res;
  }

#if DART_BUILD_MODE_DEBUG
  bool canSolve(const Eigen::MatrixXd& /*A*/) override
  {
    return true;
  }
#endif

private:
  double mValue;
};

// Secondary solver that writes impulses but reports failure.
class ConstantFailingBoxedLcpSolver : public constraint::BoxedLcpSolver
{
public:
  explicit ConstantFailingBoxedLcpSolver(double value) : mValue(value) {}

  const std::string& getType() const override
  {
    static const std::string type{"ConstantFailingBoxedLcpSolver"};
    return type;
  }

  dart::math::LcpResult solve(
      const Eigen::MatrixXd& A,
      const Eigen::VectorXd& /*b*/,
      const Eigen::VectorXd& /*lo*/,
      const Eigen::VectorXd& /*hi*/,
      const Eigen::VectorXi& /*findex*/,
      Eigen::VectorXd& x,
      const dart::math::LcpOptions& /*options*/) override
  {
    x = Eigen::VectorXd::Constant(A.rows(), mValue);
    dart::math::LcpResult res;
    res.status = dart::math::LcpSolverStatus::Failed; // intentionally fail
    return res;
  }

#if DART_BUILD_MODE_DEBUG
  bool canSolve(const Eigen::MatrixXd& /*A*/) override
  {
    return true;
  }
#endif

private:
  double mValue;
};

// Secondary solver that mixes NaN and finite outputs.
class PartialNanBoxedLcpSolver : public constraint::BoxedLcpSolver
{
public:
  explicit PartialNanBoxedLcpSolver(double value) : mValue(value) {}

  const std::string& getType() const override
  {
    static const std::string type{"PartialNanBoxedLcpSolver"};
    return type;
  }

  dart::math::LcpResult solve(
      const Eigen::MatrixXd& A,
      const Eigen::VectorXd& /*b*/,
      const Eigen::VectorXd& /*lo*/,
      const Eigen::VectorXd& /*hi*/,
      const Eigen::VectorXi& /*findex*/,
      Eigen::VectorXd& x,
      const dart::math::LcpOptions& /*options*/) override
  {
    x = Eigen::VectorXd::Constant(A.rows(), mValue);
    if (x.size() > 0)
      x[0] = std::numeric_limits<double>::quiet_NaN();
    dart::math::LcpResult res;
    res.status = dart::math::LcpSolverStatus::Success;
    return res;
  }

#if DART_BUILD_MODE_DEBUG
  bool canSolve(const Eigen::MatrixXd& /*A*/) override
  {
    return true;
  }
#endif

private:
  double mValue;
};

// Secondary solver that fails without touching x.
class FailingBoxedLcpSolver : public constraint::BoxedLcpSolver
{
public:
  const std::string& getType() const override
  {
    static const std::string type{"FailingBoxedLcpSolver"};
    return type;
  }

  dart::math::LcpResult solve(
      const Eigen::MatrixXd& /*A*/,
      const Eigen::VectorXd& /*b*/,
      const Eigen::VectorXd& /*lo*/,
      const Eigen::VectorXd& /*hi*/,
      const Eigen::VectorXi& /*findex*/,
      Eigen::VectorXd& /*x*/,
      const dart::math::LcpOptions& /*options*/) override
  {
    dart::math::LcpResult res;
    res.status = dart::math::LcpSolverStatus::Failed;
    return res;
  }

#if DART_BUILD_MODE_DEBUG
  bool canSolve(const Eigen::MatrixXd& /*A*/) override
  {
    return true;
  }
#endif
};

// Expose the protected solve entry point for testing.
class PublicBoxedLcpConstraintSolver
  : public constraint::BoxedLcpConstraintSolver
{
public:
  using constraint::BoxedLcpConstraintSolver::BoxedLcpConstraintSolver;

  void publicSolve(constraint::ConstrainedGroup& group)
  {
    solveConstrainedGroup(group);
  }
};

} // namespace

//==============================================================================
std::shared_ptr<World> createWorld()
{
  return simulation::World::create();
}

//==============================================================================
TEST(ConstraintSolver, DefaultConstactSurfaceHandler)
{
  auto world = createWorld();
  auto solver = world->getConstraintSolver();
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
}

//==============================================================================
TEST(ConstraintSolver, CustomConstactSurfaceHandler)
{
  class CustomHandler : public constraint::ContactSurfaceHandler
  {
  public:
    constraint::ContactSurfaceParams createParams(
        const collision::Contact& contact,
        const size_t numContactsOnCollisionObject) const override
    {
      auto params = ContactSurfaceHandler::createParams(
          contact, numContactsOnCollisionObject);
      params.mFirstFrictionalDirection = Eigen::Vector3d::UnitY();
      params.mContactSurfaceMotionVelocity = Eigen::Vector3d::UnitY();
      return params;
    }
  };

  auto world = createWorld();

  auto solver = world->getConstraintSolver();
  auto defaultHandler = solver->getLastContactSurfaceHandler();
  EXPECT_EQ(nullptr, defaultHandler->getParent());

  auto customHandler = std::make_shared<CustomHandler>();
  solver->addContactSurfaceHandler(customHandler);

  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(nullptr, defaultHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler->getParent());

  // try to remove nonexisting handler
  EXPECT_FALSE(
      solver->removeContactSurfaceHandler(std::make_shared<CustomHandler>()));

  EXPECT_TRUE(solver->removeContactSurfaceHandler(defaultHandler));
  EXPECT_EQ(nullptr, customHandler->getParent());
  EXPECT_EQ(customHandler, solver->getLastContactSurfaceHandler());

  // removing last handler should not be done, but we test it anyways
  // a printed error message is expected
  EXPECT_TRUE(solver->removeContactSurfaceHandler(customHandler));
  EXPECT_EQ(nullptr, customHandler->getParent());
  EXPECT_EQ(nullptr, solver->getLastContactSurfaceHandler());

  solver->addContactSurfaceHandler(defaultHandler);
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(defaultHandler, solver->getLastContactSurfaceHandler());

  solver->addContactSurfaceHandler(customHandler);
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(customHandler, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(nullptr, defaultHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler->getParent());

  auto customHandler2 = std::make_shared<CustomHandler>();
  auto customHandler3 = std::make_shared<CustomHandler>();
  solver->addContactSurfaceHandler(customHandler2);
  solver->addContactSurfaceHandler(customHandler3);
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(customHandler3, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(nullptr, defaultHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler->getParent());
  EXPECT_EQ(customHandler, customHandler2->getParent());
  EXPECT_EQ(customHandler2, customHandler3->getParent());

  EXPECT_TRUE(solver->removeContactSurfaceHandler(customHandler));
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(customHandler3, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(nullptr, defaultHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler2->getParent());
  EXPECT_EQ(customHandler2, customHandler3->getParent());

  EXPECT_TRUE(solver->removeContactSurfaceHandler(customHandler3));
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(customHandler2, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(nullptr, defaultHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler2->getParent());
  EXPECT_EQ(customHandler2, customHandler3->getParent());

  // after we break the chain at handler 2, default handler is no longer
  // reachable
  customHandler2->setParent(nullptr);
  EXPECT_FALSE(solver->removeContactSurfaceHandler(defaultHandler));
}

//==============================================================================
TEST(ConstraintSolver, ConstactSurfaceHandlerIsCalled)
{
  class ValueHandler : public constraint::ContactSurfaceHandler
  {
  public:
    ValueHandler(int value) : mValue(value)
    {
      // Do nothing
    }

    constraint::ContactSurfaceParams createParams(
        const collision::Contact& contact,
        const size_t numContactsOnCollisionObject) const override
    {
      auto params = ContactSurfaceHandler::createParams(
          contact, numContactsOnCollisionObject);
      mCalled = true;
      params.mPrimaryFrictionCoeff = mValue;

      return params;
    }

    mutable bool mCalled{false};
    int mValue{0};
  };

  auto world = createWorld();

  auto solver = world->getConstraintSolver();
  auto defaultHandler = solver->getLastContactSurfaceHandler();
  EXPECT_EQ(nullptr, defaultHandler->getParent());

  auto customHandler = std::make_shared<ValueHandler>(1);
  solver->addContactSurfaceHandler(customHandler);
  solver->removeContactSurfaceHandler(defaultHandler);

  customHandler->mCalled = false;
  auto params = solver->getLastContactSurfaceHandler()->createParams({}, 0);
  EXPECT_TRUE(customHandler->mCalled);
  EXPECT_EQ(1, params.mPrimaryFrictionCoeff);

  auto customHandler2 = std::make_shared<ValueHandler>(2);
  solver->addContactSurfaceHandler(customHandler2);

  customHandler->mCalled = customHandler2->mCalled = false;
  params = solver->getLastContactSurfaceHandler()->createParams({}, 0);
  EXPECT_TRUE(customHandler->mCalled);
  EXPECT_TRUE(customHandler2->mCalled);
  EXPECT_EQ(2, params.mPrimaryFrictionCoeff);

  // Try once more adding the same handler instance; this should be ignored.
  // If it were added, the createParams() call could get into an infinite loop
  // calling the last handler as its parent, so rather check for it.
  solver->addContactSurfaceHandler(customHandler2);

  customHandler->mCalled = customHandler2->mCalled = false;
  params = solver->getLastContactSurfaceHandler()->createParams({}, 0);
  EXPECT_TRUE(customHandler->mCalled);
  EXPECT_TRUE(customHandler2->mCalled);
  EXPECT_EQ(2, params.mPrimaryFrictionCoeff);
}

//==============================================================================
TEST(ConstraintSolver, ConstactSurfaceHandlerIgnoreParent)
{
  class IgnoreParentHandler : public constraint::ContactSurfaceHandler
  {
  public:
    IgnoreParentHandler(int value) : mValue(value)
    {
      // Do nothing
    }

    constraint::ContactSurfaceParams createParams(
        const collision::Contact& /*contact*/,
        const size_t /*numContactsOnCollisionObject*/) const override
    {
      auto params = constraint::ContactSurfaceParams{};
      mCalled = true;
      params.mPrimaryFrictionCoeff = mValue;

      return params;
    }

    mutable bool mCalled{false};
    int mValue{0};
  };

  auto world = createWorld();

  auto solver = world->getConstraintSolver();
  auto defaultHandler = solver->getLastContactSurfaceHandler();
  EXPECT_EQ(nullptr, defaultHandler->getParent());

  auto customHandler = std::make_shared<IgnoreParentHandler>(1);
  solver->addContactSurfaceHandler(customHandler);
  solver->removeContactSurfaceHandler(defaultHandler);

  customHandler->mCalled = false;
  auto params = solver->getLastContactSurfaceHandler()->createParams({}, 0);
  EXPECT_TRUE(customHandler->mCalled);
  EXPECT_EQ(1, params.mPrimaryFrictionCoeff);

  auto customHandler2 = std::make_shared<IgnoreParentHandler>(2);
  solver->addContactSurfaceHandler(customHandler2);

  customHandler->mCalled = customHandler2->mCalled = false;
  params = solver->getLastContactSurfaceHandler()->createParams({}, 0);
  EXPECT_FALSE(customHandler->mCalled);
  EXPECT_TRUE(customHandler2->mCalled);
  EXPECT_EQ(2, params.mPrimaryFrictionCoeff);
}

//==============================================================================
TEST(ConstraintSolver, LcpNanFallsBackToSecondary)
{
  auto constraintPtr = std::make_shared<DummyConstraint>();
  constraint::ConstrainedGroup group;
  group.addConstraint(constraintPtr);

  PublicBoxedLcpConstraintSolver solver;
  solver.setBoxedLcpSolver(std::static_pointer_cast<constraint::BoxedLcpSolver>(
      std::make_shared<NanBoxedLcpSolver>()));
  solver.setSecondaryBoxedLcpSolver(
      std::static_pointer_cast<constraint::BoxedLcpSolver>(
          std::make_shared<ConstantBoxedLcpSolver>(0.5)));

  solver.publicSolve(group);

  EXPECT_DOUBLE_EQ(0.5, constraintPtr->lastAppliedImpulse);
}

//==============================================================================
TEST(ConstraintSolver, LcpFailureZeroesImpulses)
{
  auto constraintPtr = std::make_shared<DummyConstraint>();
  constraint::ConstrainedGroup group;
  group.addConstraint(constraintPtr);

  PublicBoxedLcpConstraintSolver solver;
  solver.setBoxedLcpSolver(std::static_pointer_cast<constraint::BoxedLcpSolver>(
      std::make_shared<NanBoxedLcpSolver>()));
  solver.setSecondaryBoxedLcpSolver(
      std::static_pointer_cast<constraint::BoxedLcpSolver>(
          std::make_shared<FailingBoxedLcpSolver>()));

  solver.publicSolve(group);

  EXPECT_DOUBLE_EQ(0.0, constraintPtr->lastAppliedImpulse);
}

//==============================================================================
TEST(ConstraintSolver, LcpFiniteFallbackUsedEvenWhenReportingFailure)
{
  auto constraintPtr = std::make_shared<DummyConstraint>();
  constraint::ConstrainedGroup group;
  group.addConstraint(constraintPtr);

  PublicBoxedLcpConstraintSolver solver;
  solver.setBoxedLcpSolver(std::static_pointer_cast<constraint::BoxedLcpSolver>(
      std::make_shared<FailingBoxedLcpSolver>()));
  solver.setSecondaryBoxedLcpSolver(
      std::static_pointer_cast<constraint::BoxedLcpSolver>(
          std::make_shared<ConstantFailingBoxedLcpSolver>(0.7)));

  solver.publicSolve(group);

  EXPECT_DOUBLE_EQ(0.7, constraintPtr->lastAppliedImpulse);
}

//==============================================================================
TEST(ConstraintSolver, LcpPartialNanFallbackZeroesOnlyNaNEntries)
{
  auto constraintPtr = std::make_shared<MultiDummyConstraint>(2);
  constraint::ConstrainedGroup group;
  group.addConstraint(constraintPtr);

  PublicBoxedLcpConstraintSolver solver;
  solver.setBoxedLcpSolver(std::static_pointer_cast<constraint::BoxedLcpSolver>(
      std::make_shared<FailingBoxedLcpSolver>()));
  solver.setSecondaryBoxedLcpSolver(
      std::static_pointer_cast<constraint::BoxedLcpSolver>(
          std::make_shared<PartialNanBoxedLcpSolver>(0.8)));

  solver.publicSolve(group);

  ASSERT_EQ(2u, constraintPtr->lastAppliedImpulse.size());
  EXPECT_DOUBLE_EQ(0.0, constraintPtr->lastAppliedImpulse[0]);
  EXPECT_DOUBLE_EQ(0.8, constraintPtr->lastAppliedImpulse[1]);
}
