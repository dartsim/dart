/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include "helpers/gtest_utils.hpp"

#include "dart/constraint/constraint_solver.hpp"
#include "dart/constraint/contact_surface.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/math/lcp/lcp_solver.hpp"
#include "dart/simulation/world.hpp"

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
  explicit DummyConstraint(dynamics::SkeletonPtr skeleton)
    : mSkeleton(std::move(skeleton))
  {
    mDim = 1;
  }

  void update() override {}

  void getInformation(constraint::ConstraintInfo* info) override
  {
    info->lo[0] = 0.0;
    info->hi[0] = 1.0;
    info->b[0] = 0.0;
    info->w[0] = 0.0;
    info->findex[0] = -1;
  }

  void applyUnitImpulse(std::size_t) override {}

  void getVelocityChange(double* vel, bool) override
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
    return mSkeleton;
  }

  double lastAppliedImpulse{std::numeric_limits<double>::quiet_NaN()};

private:
  dynamics::SkeletonPtr mSkeleton;
};

class PhaseTrackingConstraint : public constraint::ConstraintBase
{
public:
  explicit PhaseTrackingConstraint(dynamics::SkeletonPtr skeleton)
    : mSkeleton(std::move(skeleton))
  {
    mDim = 1;
  }

  void update() override {}

  void getInformation(constraint::ConstraintInfo* info) override
  {
    phases.push_back(info->phase);
    useSplitImpulse.push_back(info->useSplitImpulse);
    ++getInformationCalls;

    info->lo[0] = 0.0;
    info->hi[0] = 0.0;
    info->b[0] = 0.0;
    info->w[0] = 0.0;
    info->findex[0] = -1;
  }

  void applyUnitImpulse(std::size_t) override {}

  void getVelocityChange(double* vel, bool) override
  {
    vel[0] = 1.0;
  }

  void excite() override {}
  void unexcite() override {}
  void applyImpulse(double*) override {}

  bool isActive() const override
  {
    return true;
  }

  dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return mSkeleton;
  }

  std::vector<constraint::ConstraintPhase> phases;
  std::vector<bool> useSplitImpulse;
  std::size_t getInformationCalls{0};

private:
  dynamics::SkeletonPtr mSkeleton;
};

class NanLcpSolver : public math::LcpSolver
{
public:
  math::LcpResult solve(
      const math::LcpProblem& problem,
      Eigen::VectorXd& x,
      const math::LcpOptions&) override
  {
    x = Eigen::VectorXd::Constant(
        problem.A.rows(), std::numeric_limits<double>::quiet_NaN());
    math::LcpResult res;
    res.status = math::LcpSolverStatus::Success;
    return res;
  }

  std::string getName() const override
  {
    return "NanLcpSolver";
  }
  std::string getCategory() const override
  {
    return "Test";
  }
};

class ConstantLcpSolver : public math::LcpSolver
{
public:
  explicit ConstantLcpSolver(double value) : mValue(value) {}

  math::LcpResult solve(
      const math::LcpProblem& problem,
      Eigen::VectorXd& x,
      const math::LcpOptions&) override
  {
    x = Eigen::VectorXd::Constant(problem.A.rows(), mValue);
    math::LcpResult res;
    res.status = math::LcpSolverStatus::Success;
    return res;
  }

  std::string getName() const override
  {
    return "ConstantLcpSolver";
  }
  std::string getCategory() const override
  {
    return "Test";
  }

private:
  double mValue;
};

class ConstantFailingLcpSolver : public math::LcpSolver
{
public:
  explicit ConstantFailingLcpSolver(double value) : mValue(value) {}

  math::LcpResult solve(
      const math::LcpProblem& problem,
      Eigen::VectorXd& x,
      const math::LcpOptions&) override
  {
    x = Eigen::VectorXd::Constant(problem.A.rows(), mValue);
    math::LcpResult res;
    res.status = math::LcpSolverStatus::Failed;
    return res;
  }

  std::string getName() const override
  {
    return "ConstantFailingLcpSolver";
  }
  std::string getCategory() const override
  {
    return "Test";
  }

private:
  double mValue;
};

class ExposedConstraintSolver : public constraint::ConstraintSolver
{
public:
  bool containsConstraint(
      const constraint::ConstConstraintBasePtr& constraint) const
  {
    return containConstraint(constraint);
  }
};

class PartialNanLcpSolver : public math::LcpSolver
{
public:
  explicit PartialNanLcpSolver(double value) : mValue(value) {}

  math::LcpResult solve(
      const math::LcpProblem& problem,
      Eigen::VectorXd& x,
      const math::LcpOptions&) override
  {
    x = Eigen::VectorXd::Constant(problem.A.rows(), mValue);
    x[0] = std::numeric_limits<double>::quiet_NaN();
    math::LcpResult res;
    res.status = math::LcpSolverStatus::Failed;
    return res;
  }

  std::string getName() const override
  {
    return "PartialNanLcpSolver";
  }
  std::string getCategory() const override
  {
    return "Test";
  }

private:
  double mValue;
};

class FailingLcpSolver : public math::LcpSolver
{
public:
  math::LcpResult solve(
      const math::LcpProblem&,
      Eigen::VectorXd&,
      const math::LcpOptions&) override
  {
    math::LcpResult res;
    res.status = math::LcpSolverStatus::Failed;
    return res;
  }

  std::string getName() const override
  {
    return "FailingLcpSolver";
  }
  std::string getCategory() const override
  {
    return "Test";
  }
};

} // namespace

//==============================================================================
TEST(ConstraintSolver, SecondarySolverUsedWhenPrimaryFails)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("dummy");
  solver.addSkeleton(skeleton);
  solver.addConstraint(std::make_shared<DummyConstraint>(skeleton));

  solver.setLcpSolver(std::make_shared<FailingLcpSolver>());
  solver.setSecondaryLcpSolver(std::make_shared<ConstantLcpSolver>(0.5));

  solver.solve();

  auto constraint
      = std::dynamic_pointer_cast<DummyConstraint>(solver.getConstraint(0));
  ASSERT_TRUE(constraint);
  EXPECT_DOUBLE_EQ(constraint->lastAppliedImpulse, 0.5);
}

//==============================================================================
TEST(ConstraintSolver, RemoveSkeletonErasesEntry)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("dummy");
  solver.addSkeleton(skeleton);

  ASSERT_EQ(solver.getSkeletons().size(), 1u);
  solver.removeSkeleton(skeleton);
  EXPECT_EQ(solver.getSkeletons().size(), 0u);
}

//==============================================================================
TEST(ConstraintSolver, SecondarySolverUsedWhenPrimaryReturnsNan)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("dummy");
  solver.addSkeleton(skeleton);
  solver.addConstraint(std::make_shared<DummyConstraint>(skeleton));

  solver.setLcpSolver(std::make_shared<NanLcpSolver>());
  solver.setSecondaryLcpSolver(std::make_shared<ConstantLcpSolver>(0.7));

  solver.solve();

  auto constraint
      = std::dynamic_pointer_cast<DummyConstraint>(solver.getConstraint(0));
  ASSERT_TRUE(constraint);
  EXPECT_DOUBLE_EQ(constraint->lastAppliedImpulse, 0.7);
}

//==============================================================================
TEST(ConstraintSolver, SecondaryIgnoredWhenPrimarySucceeds)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("dummy");
  solver.addSkeleton(skeleton);
  solver.addConstraint(std::make_shared<DummyConstraint>(skeleton));

  solver.setLcpSolver(std::make_shared<ConstantLcpSolver>(0.3));
  solver.setSecondaryLcpSolver(std::make_shared<ConstantLcpSolver>(1.0));

  solver.solve();

  auto constraint
      = std::dynamic_pointer_cast<DummyConstraint>(solver.getConstraint(0));
  ASSERT_TRUE(constraint);
  EXPECT_DOUBLE_EQ(constraint->lastAppliedImpulse, 0.3);
}

//==============================================================================
TEST(ConstraintSolver, PartialNanHandled)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("dummy");
  solver.addSkeleton(skeleton);
  solver.addConstraint(std::make_shared<DummyConstraint>(skeleton));

  solver.setLcpSolver(std::make_shared<PartialNanLcpSolver>(0.8));
  solver.setSecondaryLcpSolver(nullptr);

  solver.solve();

  auto constraint
      = std::dynamic_pointer_cast<DummyConstraint>(solver.getConstraint(0));
  ASSERT_TRUE(constraint);
  EXPECT_DOUBLE_EQ(constraint->lastAppliedImpulse, 0.0); // sanitized
}

//==============================================================================
TEST(ConstraintSolver, SplitImpulseSkipsNonContactPositionPass)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("dummy");
  solver.addSkeleton(skeleton);
  auto constraint = std::make_shared<PhaseTrackingConstraint>(skeleton);
  solver.addConstraint(constraint);

  solver.setLcpSolver(std::make_shared<ConstantLcpSolver>(0.0));
  solver.setSecondaryLcpSolver(nullptr);
  solver.setSplitImpulseEnabled(true);

  solver.solve();

  ASSERT_EQ(constraint->getInformationCalls, 1u);
  ASSERT_EQ(constraint->phases.size(), 1u);
  EXPECT_EQ(constraint->phases[0], constraint::ConstraintPhase::Velocity);
  ASSERT_EQ(constraint->useSplitImpulse.size(), 1u);
  EXPECT_TRUE(constraint->useSplitImpulse[0]);
}

//==============================================================================
TEST(ConstraintSolver, SolveWithNoSkeletonsDoesNothing)
{
  constraint::ConstraintSolver solver;

  EXPECT_EQ(solver.getSkeletons().size(), 0u);
  EXPECT_NO_THROW(solver.solve());
}

//==============================================================================
TEST(ConstraintSolver, AddDuplicateSkeletonIgnored)
{
  constraint::ConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("duplicate_test");

  solver.addSkeleton(skeleton);
  ASSERT_EQ(solver.getSkeletons().size(), 1u);

  solver.addSkeleton(skeleton);
  EXPECT_EQ(solver.getSkeletons().size(), 1u);
}

//==============================================================================
TEST(ConstraintSolver, RemoveNonExistentSkeletonNoOp)
{
  constraint::ConstraintSolver solver;
  auto skeleton1 = dynamics::Skeleton::create("skeleton1");
  auto skeleton2 = dynamics::Skeleton::create("skeleton2");

  solver.addSkeleton(skeleton1);
  ASSERT_EQ(solver.getSkeletons().size(), 1u);

  EXPECT_NO_THROW(solver.removeSkeleton(skeleton2));
  EXPECT_EQ(solver.getSkeletons().size(), 1u);
}

//==============================================================================
TEST(ConstraintSolver, SetTimeStepAffectsSolver)
{
  constraint::ConstraintSolver solver;

  EXPECT_DOUBLE_EQ(solver.getTimeStep(), 0.001);

  solver.setTimeStep(0.002);
  EXPECT_DOUBLE_EQ(solver.getTimeStep(), 0.002);

  solver.setTimeStep(0.0001);
  EXPECT_DOUBLE_EQ(solver.getTimeStep(), 0.0001);
}

//==============================================================================
TEST(ConstraintSolver, SetNullCollisionDetectorIgnored)
{
  constraint::ConstraintSolver solver;

  auto originalDetector = solver.getCollisionDetector();
  ASSERT_NE(originalDetector, nullptr);

  solver.setCollisionDetector(nullptr);
  EXPECT_EQ(solver.getCollisionDetector(), originalDetector);
}

//==============================================================================
TEST(ConstraintSolver, ContactSurfaceHandlerChain)
{
  constraint::ConstraintSolver solver;
  auto defaultHandler = solver.getLastContactSurfaceHandler();
  ASSERT_NE(defaultHandler, nullptr);

  auto handlerA = std::make_shared<constraint::ContactSurfaceHandler>();
  solver.addContactSurfaceHandler(handlerA);
  EXPECT_EQ(solver.getLastContactSurfaceHandler(), handlerA);
  EXPECT_EQ(handlerA->getParent(), defaultHandler);

  solver.addContactSurfaceHandler(handlerA);
  EXPECT_EQ(handlerA->getParent(), defaultHandler);

  auto handlerB = std::make_shared<constraint::ContactSurfaceHandler>();
  solver.addContactSurfaceHandler(handlerB);
  EXPECT_EQ(handlerB->getParent(), handlerA);

  EXPECT_TRUE(solver.removeContactSurfaceHandler(handlerB));
  EXPECT_EQ(solver.getLastContactSurfaceHandler(), handlerA);

  EXPECT_TRUE(solver.removeContactSurfaceHandler(handlerA));
  EXPECT_EQ(solver.getLastContactSurfaceHandler(), defaultHandler);

  auto missing = std::make_shared<constraint::ContactSurfaceHandler>();
  EXPECT_FALSE(solver.removeContactSurfaceHandler(missing));
  EXPECT_EQ(solver.getLastContactSurfaceHandler(), defaultHandler);
}

//==============================================================================
TEST(ConstraintSolver, AddRemoveConstraint)
{
  ExposedConstraintSolver solver;
  auto skeleton = dynamics::Skeleton::create("constraint");
  auto constraint = std::make_shared<DummyConstraint>(skeleton);

  EXPECT_EQ(solver.getNumConstraints(), 0u);
  solver.addConstraint(constraint);
  EXPECT_EQ(solver.getNumConstraints(), 1u);
  EXPECT_TRUE(solver.containsConstraint(constraint));

  solver.addConstraint(constraint);
  EXPECT_EQ(solver.getNumConstraints(), 1u);

  solver.removeConstraint(constraint);
  EXPECT_EQ(solver.getNumConstraints(), 0u);
  EXPECT_FALSE(solver.containsConstraint(constraint));

  EXPECT_NO_THROW(solver.removeConstraint(constraint));
}
